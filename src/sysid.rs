//! On-robot system-identification data collector for motor feedforward
//! constants (`Ks`, `Kv`, `Ka`).
//!
//! Works on any subsystem built from one or more [`MotorGroup`]s, each with any
//! number of motors: a drivetrain is a `left` and a `right` group, a
//! single-motor mechanism is one group of one motor. Every motor is commanded
//! the same voltage at the same time (so a drivetrain tracks straight). Each
//! group's motors are lumped into one averaged `omega` and fitted as a single
//! motor, giving one set of constants per group — e.g. the separate
//! `left_velocity_feedforward` / `right_velocity_feedforward` in
//! [`VelocityDifferentialConfig`](crate::velocity_differential::VelocityDifferentialConfig).
//!
//! It runs a **voltage staircase**: hold a constant voltage long enough for the
//! speed to settle, coast to a stop, step to the next voltage, and repeat
//! across the configured levels, each level forward and then in reverse
//! (negative voltage). The reverse steps are what let the fit separate the
//! static term `Ks` from the velocity term `Kv`: the `sign(omega)` flip is the
//! only thing that distinguishes them.
//!
//! Samples are buffered during the run and, once the whole staircase finishes,
//! printed to the console as **Desmos list literals** ready to copy-paste. The
//! dump has one block per group, in the order the groups were passed, holding
//! only that group's data lists:
//!
//! ```text
//! V=[signed volts per step]
//! S=[settled omega per step]
//! T=[time since its step started, every sample of every step]
//! W=[filtered estimator omega, every sample]
//! R=[raw unfiltered omega, every sample]
//! N=[step number (1-based, into V and S) of every sample]
//! ```
//!
//! The dump carries no fitting expressions. Those live in a Desmos template
//! graph you set up once; each run you paste one group's six lists over the
//! template's. The template's expressions:
//!
//! ```text
//! V ~ K_s sign(S) + K_v S
//! t_0 = 0.05
//! T_2 = T[T > t_0]
//! F = W[T > t_0] / S[N[T > t_0]]
//! F ~ 1 - e^{-(T_2 - c)/b}
//! (T, W/S[N])
//! (T, R/S[N])
//! K_a = K_v b
//! ```
//!
//! 1. **Steady-state fit → `Ks`, `Kv`:** `V ~ K_s sign(S) + K_v S`.
//! 2. **Transient fit → `Ka`.** Dividing each sample by its own step's settled
//!    speed (`W/S[N]`) turns every step's rise, at any voltage and in either
//!    direction, into the same `0 → 1` curve `1 - e^{-t/tau}` with
//!    `tau = Ka/Kv`. So one regression fits `tau` (`b`) across all steps at
//!    once, `K_a = K_v b` reads off `Ka`, and plotting `(T, W/S[N])` overlays
//!    every step so a bad one stands out. The slider `t_0` drops each step's
//!    first `t_0` seconds (the current-limited ramp) from the fit; `c` absorbs
//!    the resulting time shift.
//!
//! Each group gets its own graph and its own constants.
//!
//! `omega` uses the same motor-RPM → output-rad/s conversion as
//! [`MotorGroupVelocity`](crate::velocity_differential::MotorGroupVelocity), so
//! for a drivetrain the fitted `Ks`/`Kv`/`Ka` are already in the controller's
//! units and drop straight into
//! [`MotorFeedforward::new`](evian::control::loops::MotorFeedforward).
//!
//! ## Paste rules (Desmos silently breaks otherwise)
//! - Paste each list over the matching list in the template, one whole line
//!   at a time; the `#` lines are just guides. The groups reuse the same
//!   names, so fit one group at a time.
//! - Numbers are fixed-decimal — Desmos can't read scientific notation like
//!   `1.2e-3` inside a list.
//!
//! ## Collecting good data
//! - Run on the ground at competition weight, on a full battery.
//! - Keep the step voltages modest (≈2–7 V) so the motor current limit doesn't
//!   dominate each step's rise. Raise `t_0` in Desmos to skip whatever
//!   current-limited start remains.
//! - Make `hold` long enough that the speed visibly plateaus, and `rest` long
//!   enough that the mechanism fully coasts back to rest between steps.

use std::{
    cell::RefCell,
    fmt::Write,
    rc::Rc,
    time::{Duration, Instant},
};

use vexide::{
    prelude::{sleep, Motor},
    smart::motor::Gearset,
};

use crate::{
    desmos,
    motor_velocity::{live_rpm_sum, wheel_omega_from_rpm, MotorVelocityTracker},
};

/// Fraction of each step's samples (from the end) averaged for the settled
/// speed that feeds the steady-state fit.
const SETTLE_TAIL: f64 = 0.30;

/// Configuration for a [`collect`] run.
pub struct SysIdConfig {
    /// Step-voltage magnitudes to hold, in volts, in the order they're applied.
    /// Keep these modest (≈2–7 V) so the current limit doesn't dominate the
    /// rise of each step.
    pub step_voltages: &'static [f64],
    /// How long to hold each step. Long enough for the speed to settle.
    pub hold: Duration,
    /// How long to coast between steps so the mechanism returns to rest.
    pub rest: Duration,
    /// Logging period: how often a sample is buffered during each step. This
    /// controls only the density of the printed data — it does *not* affect
    /// any filter time constant. The estimator now runs in the shared
    /// background [`MotorVelocityTracker`](crate::motor_velocity::MotorVelocityTracker)
    /// at its own fixed poll rate, independent of this value, so changing it
    /// can't invalidate the fit. 5 ms gives a dense rise for the transient fit.
    pub sample_interval: Duration,
    /// Output revolutions per motor output-shaft revolution — for a drivetrain,
    /// the *same* `gear_ratio` passed to `VelocityDifferential`, so the fitted
    /// constants land in the controller's wheel-rad/s units. `1.0` for direct
    /// drive. (The estimator already reports gearset-reduced output-shaft RPM.)
    pub gear_ratio: f64,
}

impl Default for SysIdConfig {
    fn default() -> Self {
        Self {
            step_voltages: &[2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
            hold: Duration::from_millis(1000),
            rest: Duration::from_millis(1500),
            sample_interval: Duration::from_millis(5),
            gear_ratio: 1.0,
        }
    }
}

/// Motors driven and fitted as one: their speeds are averaged into a single
/// `omega`, so the group gets one set of constants however many motors it has.
pub struct MotorGroup {
    /// Labels this group's section of the dump.
    pub name: &'static str,
    /// The group's motors, one or more. Each motor's `Direction` must be set so
    /// a positive voltage moves the mechanism forward.
    pub motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
}

/// One group's output speed at one instant, in rad/s. `estimated` comes from
/// the group's [`MotorVelocityTracker`] estimator pipeline; `raw` is the motors'
/// own unfiltered velocity, kept alongside so the two can be compared in Desmos.
#[derive(Clone, Copy)]
struct Omega {
    estimated: f64,
    raw: f64,
}

/// One staircase step: its label, the signed voltage held, the sample times
/// (since the step started), and each group's speeds at those times, indexed
/// `[group][sample]` in the order the groups were passed to [`collect`].
struct Step {
    label: String,
    volts: f64,
    times: Vec<f64>,
    omegas: Vec<Vec<Omega>>,
}

/// Runs the full forward-and-reverse voltage staircase over every group at
/// once, then prints each group's data as Desmos list literals. Leaves every
/// motor stopped on return.
///
/// Every motor in every group must share `gearset`.
pub async fn collect(groups: &[MotorGroup], gearset: Gearset, config: &SysIdConfig) {
    // One background estimator per group, the same tracker the drivetrain uses.
    // They run for the entire staircase — including the `rest` coasts between
    // steps — so every step starts with warm filter windows instead of the
    // empty ones a per-step estimator would give.
    let trackers: Vec<_> = groups
        .iter()
        .map(|group| MotorVelocityTracker::new(group.motors.clone(), gearset))
        .collect();

    let mut steps = Vec::new();

    // Interleave direction per level — run each level forward then immediately
    // in reverse — so a battery sag or grip change over the run biases both
    // directions of a level equally instead of skewing all of forward against
    // all of reverse.
    for &level in config.step_voltages {
        for (phase, sign) in [("fwd", 1.0), ("rev", -1.0)] {
            let volts = sign * level;
            let (times, omegas) = run_step(groups, &trackers, volts, config).await;
            steps.push(Step {
                label: format!("{phase} {level:.1}V"),
                volts,
                times,
                omegas,
            });

            // Coast to a stop before the next step. 0 V = coast on a V5 motor.
            set_all(groups, 0.0);
            sleep(config.rest).await;
        }
    }

    // Belt and suspenders: make sure nothing is still driving before printing.
    set_all(groups, 0.0);

    desmos::print_paced(&desmos_blocks(groups, &steps)).await;
}

/// Holds `volts` on every motor for `config.hold`, sampling every group every
/// `config.sample_interval`. Returns the sample times, measured from the start
/// of this step, and each group's speeds indexed `[group][sample]`.
///
/// The estimated velocity is read from the persistent per-group
/// [`MotorVelocityTracker`]s, which run continuously across all steps, so each
/// step's filter windows are already warm when it begins.
async fn run_step(
    groups: &[MotorGroup],
    trackers: &[MotorVelocityTracker],
    volts: f64,
    config: &SysIdConfig,
) -> (Vec<f64>, Vec<Vec<Omega>>) {
    let mut times = Vec::new();
    let mut omegas = vec![Vec::new(); groups.len()];

    let start = Instant::now();
    while start.elapsed() < config.hold {
        // Short synchronous borrows only — the tracker tasks borrow these same
        // cells on their own schedule, so none may be held across the `.await`.
        set_all(groups, volts);

        for ((group, tracker), samples) in groups.iter().zip(trackers).zip(&mut omegas) {
            samples.push(Omega {
                estimated: estimated_omega(tracker, config.gear_ratio),
                raw: mean_raw_omega(group.motors.borrow_mut().as_mut(), config.gear_ratio),
            });
        }
        times.push(start.elapsed().as_secs_f64());
        sleep(config.sample_interval).await;
    }
    (times, omegas)
}

/// The collected steps as one block of Desmos data lists per group, with every
/// step's samples concatenated. The fitting expressions are not printed; they
/// live in the Desmos template described in the module docs. Each list is
/// alone on its own line with a fixed-decimal, scientific-notation-free format
/// so it pastes into Desmos cleanly; see [`desmos`](crate::desmos).
fn desmos_blocks(groups: &[MotorGroup], steps: &[Step]) -> String {
    let mut out = String::new();

    // Shared by every group: all groups were sampled together.
    let legend: Vec<String> = steps
        .iter()
        .enumerate()
        .map(|(index, step)| format!("{}={}", index + 1, step.label))
        .collect();
    let sample_count: usize = steps.iter().map(|step| step.times.len()).sum();
    let times = desmos::list(steps.iter().flat_map(|step| step.times.iter().copied()));
    let step_numbers = desmos::int_list(
        steps
            .iter()
            .enumerate()
            .flat_map(|(index, step)| std::iter::repeat_n(index + 1, step.times.len())),
    );

    for (index, group) in groups.iter().enumerate() {
        let name = group.name;
        let samples = || steps.iter().flat_map(move |step| step.omegas[index].iter());

        let _ = writeln!(
            out,
            "\n# ===== {name} ({} motors) =====",
            group.motors.borrow_mut().as_mut().len()
        );
        let _ = writeln!(out, "# steps (N): {}", legend.join(", "));
        let _ = writeln!(
            out,
            "# V and S have {} entries; T, W, R and N have {sample_count}",
            steps.len()
        );

        let _ = writeln!(out, "V={}", desmos::list(steps.iter().map(|step| step.volts)));
        let _ = writeln!(
            out,
            "S={}",
            desmos::list(steps.iter().map(|step| settled_omega(&step.omegas[index])))
        );
        let _ = writeln!(out, "T={times}");
        let _ = writeln!(out, "W={}", desmos::list(samples().map(|omega| omega.estimated)));
        let _ = writeln!(out, "R={}", desmos::list(samples().map(|omega| omega.raw)));
        let _ = writeln!(out, "N={step_numbers}");
    }

    out
}

/// Mean estimated omega over the settled tail of one group's samples.
fn settled_omega(samples: &[Omega]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let start = ((samples.len() as f64) * (1.0 - SETTLE_TAIL)) as usize;
    let tail = &samples[start.min(samples.len() - 1)..];
    tail.iter().map(|omega| omega.estimated).sum::<f64>() / tail.len() as f64
}

/// One group's estimated output omega (rad/s): the mean of its live motors,
/// excluding failed (`None`) reads, converted exactly as `MotorGroupVelocity`
/// does.
fn estimated_omega(tracker: &MotorVelocityTracker, gear_ratio: f64) -> f64 {
    let (sum, count) = tracker.with_velocities(live_rpm_sum);
    if count == 0 {
        return 0.0;
    }
    wheel_omega_from_rpm(sum / count as f64, gear_ratio)
}

/// Applies `volts` (clamped to each motor's range) to every motor of every
/// group.
fn set_all(groups: &[MotorGroup], volts: f64) {
    for group in groups {
        for motor in group.motors.borrow_mut().as_mut() {
            let limit = motor.max_voltage();
            let _ = motor.set_voltage(volts.clamp(-limit, limit));
        }
    }
}

/// Mean output angular velocity (rad/s) of one group from its motors' own
/// *unfiltered* [`Motor::velocity`], the pre-estimator baseline. Motors that
/// error out are skipped. Because each motor's `Direction` is configured so a
/// positive command moves the mechanism forward, the readings are
/// sign-consistent with the commanded voltage and can be averaged directly.
fn mean_raw_omega(motors: &[Motor], gear_ratio: f64) -> f64 {
    let mut sum_rpm = 0.0;
    let mut count = 0.0;
    for motor in motors {
        if let Ok(rpm) = motor.velocity() {
            sum_rpm += rpm;
            count += 1.0;
        }
    }
    if count == 0.0 {
        return 0.0;
    }
    wheel_omega_from_rpm(sum_rpm / count, gear_ratio)
}
