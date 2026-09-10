//! On-robot system-identification data collector for the drivetrain
//! feedforward constants (`Ks`, `Kv`, `Ka`).
//!
//! Drives the robot straight by commanding *every* drive motor at the same
//! voltage and treating both sides as one lumped system. It runs a **voltage
//! staircase**: hold a constant voltage long enough for the speed to settle,
//! coast to a stop, step to the next voltage, and repeat across the configured
//! levels — first forward, then the whole sequence again in reverse (negative
//! voltage). The reverse pass is what lets the fit separate the static term
//! `Ks` from the velocity term `Kv`: the `sign(omega)` flip is the only thing
//! that distinguishes them.
//!
//! Samples are buffered during the run and, once the whole staircase finishes,
//! printed to the console as **Desmos list literals** ready to copy-paste. Two
//! fits are done in Desmos:
//!
//! 1. **Steady-state fit → `Ks`, `Kv`.** One `(settled omega, commanded volts)`
//!    point per step:
//!
//!    ```text
//!    x_1=[settled omega per level]
//!    y_1=[matching commanded volts]
//!    ```
//!    then in Desmos: `y_1 ~ K_s sign(x_1) + K_v x_1`.
//!
//! 2. **Transient fit → `Ka`.** Each step's rise, one step at a time:
//!
//!    ```text
//!    x_1=[time since step start]
//!    y_1=[filtered estimator omega]   # fit this
//!    z_1=[raw unfiltered omega]       # plotted as a sanity check, not fitted
//!    ```
//!    then in Desmos: `y_1 ~ a(1 - e^{-x_1/b})`, where `b` is the time constant
//!    `tau` and `Ka = Kv * tau` (from `tau = Ka/Kv`). Fit each step's block on
//!    its own and take the median `tau` across steps.
//!
//! `omega` uses the same motor-RPM → wheel-rad/s conversion as
//! [`MotorGroupVelocity`](crate::velocity_differential::MotorGroupVelocity), so
//! the fitted `Ks`/`Kv`/`Ka` are already in the controller's units and drop
//! straight into [`MotorFeedforward::new`](evian::control::loops::MotorFeedforward).
//!
//! ## Paste rules (Desmos silently breaks otherwise)
//! - Every list is on its own line and starts with `x_1=[`, `y_1=[`, or `z_1=[`
//!   — copy one whole line at a time; the `# ...` label lines are just guides,
//!   don't paste them.
//! - Paste one block's lists into a fresh Desmos before fitting; the reused
//!   `x_1`/`y_1`/`z_1` names collide if you paste two blocks at once.
//! - Numbers are fixed-decimal — Desmos can't read scientific notation like
//!   `1.2e-3` inside a list.
//!
//! ## Collecting good data
//! - Run on the ground at competition weight, on a full battery.
//! - Keep the step voltages modest (≈2–7 V) so the motor current limit doesn't
//!   dominate each step's rise. When fitting a transient in Desmos, restrict the
//!   domain to skip the current-limited start and the noisy settled tail.
//! - Make `hold` long enough that the speed visibly plateaus, and `rest` long
//!   enough that the robot fully coasts back to rest between steps.

use std::{
    f64::consts::PI,
    time::{Duration, Instant},
};

use vexide::{
    math::Direction,
    prelude::{sleep, Motor},
};

use crate::{
    sensor::{TimestampedPosition, MOTOR_RAW_POSITION_RESPECTS_DIRECTION},
    velocity_estimator::VelocityEstimator,
};

/// Fallback output-shaft free speed (blue cartridge) if a motor's gearset can't
/// be read while building its estimator.
const DEFAULT_GEARSET_RPM: f64 = 600.0;

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
    /// How long to coast between steps so the robot returns to rest.
    pub rest: Duration,
    /// Sampling period. Must match
    /// [`MotorVelocityTracker`](crate::motor_velocity::MotorVelocityTracker)'s poll
    /// rate (`Motor::UPDATE_INTERVAL / 2` = 5 ms) so the estimator's sample-count
    /// filter windows have identical time constants during identification and in
    /// the control loop. Changing it invalidates the fitted `Kv`/`Ka`.
    pub sample_interval: Duration,
    /// Wheel revolutions per motor output-shaft revolution — the *same*
    /// `gear_ratio` passed to `VelocityDifferential`, so the fitted constants
    /// land in the controller's wheel-rad/s units. `1.0` for direct drive.
    /// (The estimator already reports gearset-reduced output-shaft RPM.)
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

/// One staircase step: its label, the signed voltage held, and the buffered
/// `(t, estimated_omega, raw_omega)` samples of the rise. `estimated_omega` comes
/// from the [`VelocityEstimator`] pipeline; `raw_omega` is the motor's own
/// unfiltered velocity, kept alongside so the two can be compared in Desmos.
struct Step {
    label: String,
    volts: f64,
    samples: Vec<(f64, f64, f64)>,
}

/// Runs the full forward-then-reverse voltage staircase, then prints the
/// collected data as Desmos list literals. Leaves every motor stopped on
/// return.
///
/// `left` and `right` are the two drive sides. They're commanded identically
/// (equal voltage) so the robot tracks straight; the two sides are lumped into
/// a single averaged `omega` measurement.
pub async fn collect(left: &mut [Motor], right: &mut [Motor], config: &SysIdConfig) {
    let mut steps = Vec::new();

    // Interleave direction per level — run each level forward then immediately
    // in reverse — so a battery sag or grip change over the run biases both
    // directions of a level equally instead of skewing all of forward against
    // all of reverse.
    for &level in config.step_voltages {
        for (phase, sign) in [("fwd", 1.0), ("rev", -1.0)] {
            let volts = sign * level;
            let samples = run_step(left, right, volts, config).await;
            steps.push(Step {
                label: format!("{phase} {level:.1}V"),
                volts,
                samples,
            });

            // Coast to a stop before the next step. 0 V = coast on a V5 motor.
            set_all(left, right, 0.0);
            sleep(config.rest).await;
        }
    }

    // Belt and suspenders: make sure nothing is still driving before printing.
    set_all(left, right, 0.0);

    print_desmos(&steps);
}

/// Holds `volts` on every motor for `config.hold`, buffering a
/// `(t, estimated_omega, raw_omega)` sample every `config.sample_interval`. `t`
/// is measured from the start of this step.
///
/// A fresh [`VelocityEstimator`] per motor is built for each step so the filter
/// state doesn't carry across the coast between steps.
async fn run_step(
    left: &mut [Motor],
    right: &mut [Motor],
    volts: f64,
    config: &SysIdConfig,
) -> Vec<(f64, f64, f64)> {
    let mut samples = Vec::new();
    let mut estimators = build_estimators(left, right);
    let mut velocities = vec![0.0; estimators.len()];

    let start = Instant::now();
    while start.elapsed() < config.hold {
        set_all(left, right, volts);
        update_estimators(left, right, &mut estimators, &mut velocities);
        let estimated = mean_omega(&velocities, config.gear_ratio);
        let raw = mean_raw_omega(left, right, config.gear_ratio);
        let t = start.elapsed().as_secs_f64();
        samples.push((t, estimated, raw));
        sleep(config.sample_interval).await;
    }
    samples
}

/// One [`VelocityEstimator`] per drive motor (left then right), each seeded with
/// its motor's gearset free speed.
fn build_estimators(left: &[Motor], right: &[Motor]) -> Vec<VelocityEstimator> {
    left.iter()
        .chain(right.iter())
        .map(|motor| {
            let gearset_rpm = motor
                .gearset()
                .map(|gearset| gearset.max_rpm())
                .unwrap_or(DEFAULT_GEARSET_RPM);
            VelocityEstimator::new(gearset_rpm)
        })
        .collect()
}

/// Feeds one timestamped sample into every estimator, writing the resulting
/// per-motor output-shaft RPM into `velocities` (left then right). Motors that
/// error out keep their previous value.
fn update_estimators(
    left: &[Motor],
    right: &[Motor],
    estimators: &mut [VelocityEstimator],
    velocities: &mut [f64],
) {
    for (index, motor) in left.iter().chain(right.iter()).enumerate() {
        let Ok((ticks, timestamp)) = motor.timestamped_position() else {
            continue;
        };
        let mut rpm = estimators[index].update(ticks, timestamp);
        if !MOTOR_RAW_POSITION_RESPECTS_DIRECTION
            && matches!(motor.direction(), Ok(Direction::Reverse))
        {
            rpm = -rpm;
        }
        velocities[index] = rpm;
    }
}

/// Prints the collected steps as Desmos list literals: one steady-state block
/// for `Ks`/`Kv`, then one transient block per step for `Ka`. Each list is
/// alone on its own line with a fixed-decimal, scientific-notation-free format
/// so it pastes into Desmos cleanly.
fn print_desmos(steps: &[Step]) {
    // --- Steady-state fit: settled omega vs commanded volts, one per level ---
    println!();
    println!("# steady-state fit -> Ks, Kv   (paste both lists, then:  y_1 ~ K_s sign(x_1) + K_v x_1)");
    let omegas: Vec<String> = steps
        .iter()
        .map(|step| format!("{:.4}", settled_omega(&step.samples)))
        .collect();
    let volts: Vec<String> = steps.iter().map(|step| format!("{:.4}", step.volts)).collect();
    println!("x_1=[{}]", omegas.join(","));
    println!("y_1=[{}]", volts.join(","));

    // --- Transient fits: one step at a time -> tau, then Ka = Kv * tau ---
    // y_1 is the filtered estimator omega (fit this); z_1 is the raw unfiltered
    // omega, plotted alongside as a sanity check on the estimator.
    for step in steps {
        println!();
        println!(
            "# transient {} -> tau=b, Ka=Kv*b   (fit y_1 ~ a(1 - e^{{-x_1/b}}); z_1 is raw omega)",
            step.label
        );
        let ts: Vec<String> = step.samples.iter().map(|(t, ..)| format!("{t:.4}")).collect();
        let ws: Vec<String> = step
            .samples
            .iter()
            .map(|(_, estimated, _)| format!("{estimated:.4}"))
            .collect();
        let raws: Vec<String> = step
            .samples
            .iter()
            .map(|(.., raw)| format!("{raw:.4}"))
            .collect();
        println!("x_1=[{}]", ts.join(","));
        println!("y_1=[{}]", ws.join(","));
        println!("z_1=[{}]", raws.join(","));
    }
}

/// Mean estimated omega over the settled tail of a step's samples.
fn settled_omega(samples: &[(f64, f64, f64)]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let start = ((samples.len() as f64) * (1.0 - SETTLE_TAIL)) as usize;
    let tail = &samples[start.min(samples.len() - 1)..];
    tail.iter().map(|(_, estimated, _)| estimated).sum::<f64>() / tail.len() as f64
}

/// Applies `volts` (clamped to each motor's range) to every drive motor.
fn set_all(left: &mut [Motor], right: &mut [Motor], volts: f64) {
    for motor in left.iter_mut().chain(right.iter_mut()) {
        let limit = motor.max_voltage();
        let _ = motor.set_voltage(volts.clamp(-limit, limit));
    }
}

/// Mean wheel angular velocity (rad/s) from the estimator pipeline's per-motor
/// output-shaft RPM in `velocities`, converting to wheel rad/s exactly as
/// `MotorGroupVelocity` does.
fn mean_omega(velocities: &[f64], gear_ratio: f64) -> f64 {
    if velocities.is_empty() {
        return 0.0;
    }
    let mean_rpm = velocities.iter().sum::<f64>() / velocities.len() as f64;
    mean_rpm * gear_ratio * (2.0 * PI / 60.0)
}

/// Mean wheel angular velocity (rad/s) from the motors' own *unfiltered*
/// [`Motor::velocity`], the pre-estimator baseline. Motors that error out are
/// skipped. Because each motor's `Direction` is configured so a positive command
/// drives the robot forward, the readings are sign-consistent with the commanded
/// voltage and can be averaged directly.
fn mean_raw_omega(left: &[Motor], right: &[Motor], gear_ratio: f64) -> f64 {
    let mut sum_rpm = 0.0;
    let mut count = 0.0;
    for motor in left.iter().chain(right.iter()) {
        if let Ok(rpm) = motor.velocity() {
            sum_rpm += rpm;
            count += 1.0;
        }
    }
    if count == 0.0 {
        return 0.0;
    }
    (sum_rpm / count) * gear_ratio * (2.0 * PI / 60.0)
}
