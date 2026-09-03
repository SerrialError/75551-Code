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
//!    y_1=[measured omega]
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
//! - Every list is on its own line and starts with `x_1=[` / `y_1=[` — copy one
//!   whole line at a time; the `# ...` label lines are just guides, don't paste
//!   them.
//! - Paste one block (its `x_1` and `y_1`) into a fresh Desmos before fitting;
//!   the reused `x_1`/`y_1` names collide if you paste two blocks at once.
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

use vexide::prelude::{sleep, Motor};

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
    /// Logging period. ≈10 ms gives the target ~100 Hz.
    pub sample_interval: Duration,
    /// Wheel revolutions per motor output-shaft revolution — the *same*
    /// `gear_ratio` passed to `VelocityDifferential`, so the fitted constants
    /// land in the controller's wheel-rad/s units. `1.0` for direct drive.
    /// ([`Motor::velocity`] already reports gearset-reduced RPM.)
    pub gear_ratio: f64,
}

impl Default for SysIdConfig {
    fn default() -> Self {
        Self {
            step_voltages: &[2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
            hold: Duration::from_millis(1500),
            rest: Duration::from_millis(1500),
            sample_interval: Duration::from_millis(10),
            gear_ratio: 1.0,
        }
    }
}

/// One staircase step: its label, the signed voltage held, and the buffered
/// `(t, omega)` samples of the rise.
struct Step {
    label: String,
    volts: f64,
    samples: Vec<(f64, f64)>,
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

    for (phase, sign) in [("fwd", 1.0), ("rev", -1.0)] {
        for &level in config.step_voltages {
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

/// Holds `volts` on every motor for `config.hold`, buffering a `(t, omega)`
/// sample every `config.sample_interval`. `t` is measured from the start of
/// this step.
async fn run_step(
    left: &mut [Motor],
    right: &mut [Motor],
    volts: f64,
    config: &SysIdConfig,
) -> Vec<(f64, f64)> {
    let mut samples = Vec::new();
    let start = Instant::now();
    while start.elapsed() < config.hold {
        set_all(left, right, volts);
        let omega = mean_omega(left, right, config.gear_ratio);
        let t = start.elapsed().as_secs_f64();
        samples.push((t, omega));
        sleep(config.sample_interval).await;
    }
    samples
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
    for step in steps {
        println!();
        println!(
            "# transient {} -> tau=b, Ka=Kv*b   (paste both lists, then:  y_1 ~ a(1 - e^{{-x_1/b}}))",
            step.label
        );
        let ts: Vec<String> = step.samples.iter().map(|(t, _)| format!("{t:.4}")).collect();
        let ws: Vec<String> = step.samples.iter().map(|(_, w)| format!("{w:.4}")).collect();
        println!("x_1=[{}]", ts.join(","));
        println!("y_1=[{}]", ws.join(","));
    }
}

/// Mean omega over the settled tail of a step's samples.
fn settled_omega(samples: &[(f64, f64)]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let start = ((samples.len() as f64) * (1.0 - SETTLE_TAIL)) as usize;
    let tail = &samples[start.min(samples.len() - 1)..];
    tail.iter().map(|(_, w)| w).sum::<f64>() / tail.len() as f64
}

/// Applies `volts` (clamped to each motor's range) to every drive motor.
fn set_all(left: &mut [Motor], right: &mut [Motor], volts: f64) {
    for motor in left.iter_mut().chain(right.iter_mut()) {
        let limit = motor.max_voltage();
        let _ = motor.set_voltage(volts.clamp(-limit, limit));
    }
}

/// Mean wheel angular velocity (rad/s) across every drive motor, converting
/// motor-output RPM to wheel rad/s exactly as `MotorGroupVelocity` does. Motors
/// that error out are skipped. Because each motor's `Direction` is configured so
/// a positive command drives the robot forward, the readings are sign-consistent
/// with the commanded voltage and can be averaged directly.
fn mean_omega(left: &[Motor], right: &[Motor], gear_ratio: f64) -> f64 {
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
