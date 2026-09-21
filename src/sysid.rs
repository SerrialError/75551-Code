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
    /// How long to coast between steps so the robot returns to rest.
    pub rest: Duration,
    /// Logging period: how often a `(t, omega)` sample is buffered during each
    /// step. This controls only the density of the printed data — it does *not*
    /// affect any filter time constant. The estimator now runs in the shared
    /// background [`MotorVelocityTracker`](crate::motor_velocity::MotorVelocityTracker)
    /// at its own fixed poll rate, independent of this value, so changing it
    /// can't invalidate the fit. 5 ms gives a dense rise for the transient fit.
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
/// from the per-side [`MotorVelocityTracker`] estimator pipeline; `raw_omega` is
/// the motor's own unfiltered velocity, kept alongside so the two can be compared
/// in Desmos.
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
pub async fn collect(
    left: Rc<RefCell<dyn AsMut<[Motor]>>>,
    right: Rc<RefCell<dyn AsMut<[Motor]>>>,
    gearset: Gearset,
    config: &SysIdConfig,
) {
    // One background estimator per side, the same tracker the drivetrain uses.
    // They run for the entire staircase — including the `rest` coasts between
    // steps — so every step starts with warm filter windows instead of the
    // empty ones a per-step estimator would give.
    let left_tracker = MotorVelocityTracker::new(left.clone(), gearset);
    let right_tracker = MotorVelocityTracker::new(right.clone(), gearset);

    let mut steps = Vec::new();

    // Interleave direction per level — run each level forward then immediately
    // in reverse — so a battery sag or grip change over the run biases both
    // directions of a level equally instead of skewing all of forward against
    // all of reverse.
    for &level in config.step_voltages {
        for (phase, sign) in [("fwd", 1.0), ("rev", -1.0)] {
            let volts = sign * level;
            let samples = run_step(&left, &right, &left_tracker, &right_tracker, volts, config).await;
            steps.push(Step {
                label: format!("{phase} {level:.1}V"),
                volts,
                samples,
            });

            // Coast to a stop before the next step. 0 V = coast on a V5 motor.
            set_all(left.borrow_mut().as_mut(), right.borrow_mut().as_mut(), 0.0);
            sleep(config.rest).await;
        }
    }

    // Belt and suspenders: make sure nothing is still driving before printing.
    set_all(left.borrow_mut().as_mut(), right.borrow_mut().as_mut(), 0.0);

    print_desmos(&steps);
}

/// Holds `volts` on every motor for `config.hold`, buffering a
/// `(t, estimated_omega, raw_omega)` sample every `config.sample_interval`. `t`
/// is measured from the start of this step.
///
/// The estimated velocity is read from the persistent per-side
/// [`MotorVelocityTracker`]s, which run continuously across all steps, so each
/// step's filter windows are already warm when it begins.
async fn run_step(
    left: &Rc<RefCell<dyn AsMut<[Motor]>>>,
    right: &Rc<RefCell<dyn AsMut<[Motor]>>>,
    left_tracker: &MotorVelocityTracker,
    right_tracker: &MotorVelocityTracker,
    volts: f64,
    config: &SysIdConfig,
) -> Vec<(f64, f64, f64)> {
    let mut samples = Vec::new();

    let start = Instant::now();
    while start.elapsed() < config.hold {
        // Short synchronous borrows only — the tracker task borrows these same
        // cells on its own schedule, so none may be held across the `.await`.
        set_all(left.borrow_mut().as_mut(), right.borrow_mut().as_mut(), volts);

        // Lump both sides' live motors into one mean, excluding failed (`None`)
        // reads, then convert exactly as `MotorGroupVelocity` does.
        let (left_sum, left_count) = left_tracker.with_velocities(live_rpm_sum);
        let (right_sum, right_count) = right_tracker.with_velocities(live_rpm_sum);
        let count = left_count + right_count;
        let estimated = if count == 0 {
            0.0
        } else {
            wheel_omega_from_rpm((left_sum + right_sum) / count as f64, config.gear_ratio)
        };

        let raw = mean_raw_omega(
            left.borrow_mut().as_mut(),
            right.borrow_mut().as_mut(),
            config.gear_ratio,
        );
        let t = start.elapsed().as_secs_f64();
        samples.push((t, estimated, raw));
        sleep(config.sample_interval).await;
    }
    samples
}

/// Prints what [`desmos_blocks`] formatted.
fn print_desmos(steps: &[Step]) {
    print!("{}", desmos_blocks(steps));
}

/// The collected steps as Desmos list literals: one steady-state block for
/// `Ks`/`Kv`, then one transient block per step for `Ka`. Each list is alone on
/// its own line with a fixed-decimal, scientific-notation-free format so it
/// pastes into Desmos cleanly; see [`desmos`](crate::desmos).
fn desmos_blocks(steps: &[Step]) -> String {
    let mut out = String::new();

    // --- Steady-state fit: settled omega vs commanded volts, one per level ---
    // x_1 is the settled omega of each step and y_1 the volts that held it, in
    // matching order, which is the pair the module docs' first example fits.
    out.push('\n');
    out.push_str(
        "# steady-state fit -> Ks, Kv   (paste both lists, then:  y_1 ~ K_s sign(x_1) + K_v x_1)\n",
    );
    let _ = writeln!(
        out,
        "x_1={}",
        desmos::list(steps.iter().map(|step| settled_omega(&step.samples)))
    );
    let _ = writeln!(
        out,
        "y_1={}",
        desmos::list(steps.iter().map(|step| step.volts))
    );

    // --- Transient fits: one step at a time -> tau, then Ka = Kv * tau ---
    // y_1 is the filtered estimator omega (fit this); z_1 is the raw unfiltered
    // omega, plotted alongside as a sanity check on the estimator.
    for step in steps {
        let _ = writeln!(
            out,
            "\n# transient {} -> tau=b, Ka=Kv*b   (fit y_1 ~ a(1 - e^{{-x_1/b}}); z_1 is raw omega)",
            step.label
        );
        let _ = writeln!(
            out,
            "x_1={}",
            desmos::list(step.samples.iter().map(|&(t, ..)| t))
        );
        let _ = writeln!(
            out,
            "y_1={}",
            desmos::list(step.samples.iter().map(|&(_, estimated, _)| estimated))
        );
        let _ = writeln!(
            out,
            "z_1={}",
            desmos::list(step.samples.iter().map(|&(.., raw)| raw))
        );
    }

    out
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
    wheel_omega_from_rpm(sum_rpm / count, gear_ratio)
}

#[cfg(test)]
mod tests {
    use super::{desmos_blocks, Step};

    fn step(label: &str, volts: f64, samples: Vec<(f64, f64, f64)>) -> Step {
        Step {
            label: label.to_string(),
            volts,
            samples,
        }
    }

    /// The module docs promise a steady-state block of one settled omega per
    /// level in `x_1` against the volts that held it in `y_1`; that pairing is
    /// what `y_1 ~ K_s sign(x_1) + K_v x_1` fits, so it is worth pinning.
    #[test]
    fn steady_state_block_pairs_omega_against_volts() {
        // A flat step settles at its own omega, whatever `SETTLE_TAIL` keeps.
        let steps = [
            step("fwd 2.0V", 2.0, vec![(0.0, 10.0, 10.0); 4]),
            step("rev 2.0V", -2.0, vec![(0.0, -10.0, -10.0); 4]),
        ];
        let blocks = desmos_blocks(&steps);

        assert!(blocks.contains("\nx_1=[10.0000,-10.0000]\n"));
        assert!(blocks.contains("\ny_1=[2.0000,-2.0000]\n"));
    }

    #[test]
    fn each_step_gets_its_own_transient_block() {
        let steps = [step(
            "fwd 2.0V",
            2.0,
            vec![(0.0, 0.0, 0.1), (0.005, 1.0, 1.2)],
        )];
        let blocks = desmos_blocks(&steps);

        assert!(blocks.contains("# transient fwd 2.0V"));
        assert!(blocks.contains("\nx_1=[0.0000,0.0050]\n"));
        assert!(blocks.contains("\ny_1=[0.0000,1.0000]\n"));
        assert!(blocks.contains("\nz_1=[0.1000,1.2000]\n"));
    }
}
