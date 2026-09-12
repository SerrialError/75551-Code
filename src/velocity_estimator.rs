//! A sylib-style motor velocity estimator.
//!
//! V5 Smart motors report an internally-estimated velocity ([`Motor::velocity`]),
//! but that estimate is noisy and laggy at the speeds a drivetrain velocity loop
//! cares about. This estimator instead differentiates the motor's raw encoder
//! position against the Brain's clock and runs the result through a small
//! filter chain, closely following sylib's approach:
//! <https://sylvie.fyi/sylib/docs/db/d8e/md_module_writeups__velocity__estimation.html>
//!
//! The filter chain follows sylib; the dT handling deliberately does not. sylib
//! corrects dt using a per-sample motor timestamp, quantizing it to 5 ms
//! multiples to undo the motor's ~5 ms sampling straddle. That correction is
//! omitted here: V5 motors transmit no timestamp of their own, so the straddle
//! it compensates for is not observable from the Brain's clock. Re-adding it
//! would only distort dt whenever a packet publish is missed. (This has been
//! mistakenly re-added once — leave it out.)
//!
//! The pipeline, per [`update`](VelocityEstimator::update):
//!
//! 1. A raw RPM from the tick/time difference (at the motor's *internal* shaft).
//! 2. A 3-tap moving average to knock down encoder quantization noise.
//! 3. A 7-tap median off the smoothed value to reject single-sample spikes.
//! 4. A derivative of the median (an acceleration estimate) whose recent peak
//!    magnitude drives an adaptive EMA gain: the filter tracks quickly during
//!    acceleration transients and smooths hard when the speed is steady.
//!
//! All time in this file is in **milliseconds**. The adaptive-gain constants are
//! calibrated to that scale, so do not convert to seconds anywhere here.
//!
//! # Startup
//!
//! The filters operate on partial windows, so the first ~20 samples are
//! progressively smoothed rather than fully filtered. This is intentional and
//! matches sylib, which divides by the actual sample count. The alternative —
//! reporting zero until every window fills — would emit ~200 ms of false zero
//! velocity that a feedback loop would integrate as real error.
//!
//! This module has no `vexide`/hardware dependency and is unit-tested on the
//! host; the caller supplies `(ticks, timestamp_ms)` (see
//! [`TimestampedPosition`](crate::sensor::TimestampedPosition)).

use crate::filters::{Derivative, Ema, MaxAbs, Median, Sma};

/// Raw encoder ticks per revolution of the motor's *internal* (pre-gearset)
/// shaft.
//
// 50 ticks per revolution of the 3600 RPM internal rotor. Confirmed by VEX's
// published per-output-revolution figures (1800 at 36:1, 900 at 18:1, 300 at
// 6:1), which are also vexide's `Gearset::*_TICKS_PER_REVOLUTION` constants —
// each divided by its reduction gives 50. Note that vexide's `raw_position` doc
// comment claims a TPR of 4096; that contradicts vexide's own gearset constants
// and is wrong.
const TICKS_PER_INTERNAL_REV: f64 = 50.0;

/// Free speed of the motor's internal shaft, in RPM. Output-shaft RPM is
/// internal RPM scaled by `GEARSET_RPM / INTERNAL_FREE_SPEED_RPM`.
const INTERNAL_FREE_SPEED_RPM: f64 = 3600.0;

/// Any raw RPM whose magnitude exceeds this is treated as a spurious position
/// reset (e.g. `raw_position()` being re-zeroed) rather than real motion.
const MAX_PLAUSIBLE_RAW_RPM: f64 = 5000.0;

/// EMA gain approached under hard acceleration.
const GAIN_MAX: f64 = 0.75;

/// EMA gain at zero acceleration — heavy steady-state smoothing.
///
/// At the estimator's ~10 ms effective sample rate this is a time constant near
/// one second.
///
// TODO: this is probably too low for a drivetrain velocity loop. These constants
// come from sylib, where they were tuned empirically for flywheels — a plant
// that is slow anyway and cares far more about steady-state accuracy than
// latency. A drivetrain velocity loop is the opposite trade: ~1 s of lag in the
// feedback path forces the velocity PID's gains down far enough to partly defeat
// having the loop at all.
//
// The adaptive gain rescues large transients, which is its purpose. It does not
// rescue small disturbances near steady state, where `peak` stays low, the gain
// sits at this floor, and the loop reacts to information up to a second stale.
//
// Check this against the sysid traces: if `y_1` visibly lags into the flat
// region while `z_1` has already settled, the floor is too low. First thing to
// try is raising this to ~0.05 (a ~200 ms time constant) and re-running.
const GAIN_MIN: f64 = 0.0096;

/// Knee of the gain curve, in (RPM/ms)². The gain sits midway between
/// [`GAIN_MIN`] and [`GAIN_MAX`] when peak acceleration is `sqrt` of this
/// (≈ 7.1 RPM/ms, about 1180 output RPM/s on a blue cartridge).
///
/// Also from sylib, also empirical — there is no derivation behind it, and this
/// drivetrain has no particular reason to want the same knee.
const ACCEL_SCALE: f64 = 50.0;

/// Estimates a single motor's output-shaft velocity from timestamped raw
/// encoder samples.
pub struct VelocityEstimator {
    /// Motor free speed at the output shaft for this motor's gearset, in RPM
    /// (e.g. 600 for a blue cartridge).
    gearset_rpm: f64,

    sma_3: Sma,
    median_7: Median,
    derivative: Derivative,
    max_abs_20: MaxAbs,
    ema: Ema,

    previous_ticks: i32,
    previous_timestamp_ms: u32,
    last_output: f64,
    seeded: bool,
}

impl VelocityEstimator {
    /// `gearset_rpm` is the motor's output-shaft free speed for its gearset
    /// (blue = 600).
    pub fn new(gearset_rpm: f64) -> Self {
        Self {
            gearset_rpm,
            sma_3: Sma::new(3),
            median_7: Median::new(7),
            derivative: Derivative::new(),
            max_abs_20: MaxAbs::new(20),
            ema: Ema::new(),
            previous_ticks: 0,
            previous_timestamp_ms: 0,
            last_output: 0.0,
            seeded: false,
        }
    }

    /// Feeds one timestamped raw-encoder sample and returns the estimated motor
    /// output-shaft velocity in RPM.
    ///
    /// `ticks` is the raw (pre-gearset) encoder count and `timestamp_ms` is the
    /// Brain's clock reading in milliseconds.
    pub fn update(&mut self, ticks: i32, timestamp_ms: u32) -> f64 {
        // The first sample only establishes a baseline to difference against.
        if !self.seeded {
            self.previous_ticks = ticks;
            self.previous_timestamp_ms = timestamp_ms;
            self.seeded = true;
            return self.last_output;
        }

        // 1. dt from the Brain's clock. No elapsed time -> nothing new to say.
        let dt = timestamp_ms.wrapping_sub(self.previous_timestamp_ms);
        if dt == 0 {
            return self.last_output;
        }
        let dt = dt as f64;

        // 2. Raw internal-shaft RPM: revs over the interval, scaled to per-minute.
        let delta_ticks = (ticks as i64 - self.previous_ticks as i64) as f64;

        // Advance the baseline *before* the reset guard below can bail out.
        // Otherwise a real `reset_position()` wedges the estimator forever: every
        // later sample would difference against the stale pre-reset ticks, so
        // `raw_rpm` would stay above the threshold and we'd never output again.
        self.previous_ticks = ticks;
        self.previous_timestamp_ms = timestamp_ms;

        let raw_rpm = (delta_ticks / TICKS_PER_INTERNAL_REV) / dt * 60_000.0;

        // 3. An implausible jump means the encoder was reset, not that the motor
        //    briefly hit thousands of RPM: drop the sample. The baseline was
        //    already advanced above, so the next sample differences against this
        //    one; only the filters and `last_output` are left untouched.
        if raw_rpm.abs() > MAX_PLAUSIBLE_RAW_RPM {
            return self.last_output;
        }

        // 4. Smooth the quantization noise.
        let smoothed = self.sma_3.filter(raw_rpm);
        // 5. Reject single-sample spikes for the acceleration estimate.
        let median = self.median_7.filter(smoothed);
        // 6. Acceleration estimate (RPM per millisecond)...
        let accel = self.derivative.filter(median, dt);
        // 7. ...and its recent peak magnitude.
        let peak = self.max_abs_20.filter(accel);        
        // 8. Adaptive gain: a saturating curve from GAIN_MIN at rest toward
        //    GAIN_MAX under acceleration, so the estimate tracks quickly through
        //    transients and smooths hard when the speed is steady. `shape` is the
        //    constant that places the floor at GAIN_MIN; it is derived from the
        //    other two rather than tuned. See GAIN_MIN's note on the floor being
        //    likely too low for a drivetrain.
        let shape = GAIN_MAX / (GAIN_MAX - GAIN_MIN);
        let gain = GAIN_MAX * (1.0 - 1.0 / ((peak * peak / ACCEL_SCALE) + shape));
        // 9. EMA the *smoothed* value with the adaptive gain, then convert
        //    internal-shaft RPM to output-shaft RPM.
        let output =
            self.ema.filter(smoothed, gain) * self.gearset_rpm / INTERNAL_FREE_SPEED_RPM;

        self.last_output = output;
        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Blue-cartridge free speed used across the tests.
    const BLUE_RPM: f64 = 600.0;

    #[test]
    fn first_sample_only_seeds_and_returns_zero() {
        let mut estimator = VelocityEstimator::new(BLUE_RPM);
        assert_eq!(estimator.update(0, 100), 0.0);
    }

    #[test]
    fn zero_dt_returns_last_output_unchanged() {
        let mut estimator = VelocityEstimator::new(BLUE_RPM);
        estimator.update(0, 100);
        let a = estimator.update(50, 110);
        // Same timestamp -> dt == 0 -> unchanged.
        let b = estimator.update(999, 110);
        assert_eq!(a, b);
    }

    #[test]
    fn position_reset_spike_is_ignored() {
        let mut estimator = VelocityEstimator::new(BLUE_RPM);
        estimator.update(0, 0);
        // 30 ticks / 10 ms = 3600 internal RPM: plausible motion.
        let before = estimator.update(30, 10);
        // A huge tick jump over a short interval is an encoder reset, not motion:
        // the sample is dropped and the last output held.
        let after = estimator.update(1_000_000, 20);
        assert_eq!(before, after);

        // Because the baseline advanced to the reset value, normal motion resumes
        // immediately: the next samples difference against 1_000_000, not the
        // stale pre-reset ticks, so the estimator recovers instead of wedging.
        let recovered = estimator.update(1_000_030, 30);
        let recovered = estimator.update(1_000_060, 40).max(recovered);
        assert!(
            recovered > 1.0,
            "estimator should recover after a reset, got {recovered}"
        );
    }

    #[test]
    fn steady_state_converges_to_expected_output_rpm() {
        // Spin the internal shaft at a constant 30 ticks / 10 ms.
        // 30 ticks / 50 ticks-per-rev = 0.6 rev per 10 ms = 3600 internal RPM,
        // which at the blue gearset is 3600 * 600 / 3600 = 600 output RPM. The
        // steady-state gain is small, so run long enough for the EMA to settle.
        let mut estimator = VelocityEstimator::new(BLUE_RPM);
        let mut ticks = 0i32;
        let mut t = 0u32;
        let mut output = 0.0;
        for _ in 0..3000 {
            ticks += 30;
            t += 10;
            output = estimator.update(ticks, t);
        }
        assert!(
            (output - 600.0).abs() < 1.0,
            "expected ~600 output RPM, got {output}"
        );
    }

    #[test]
    fn output_is_zero_when_stationary() {
        let mut estimator = VelocityEstimator::new(BLUE_RPM);
        let mut output = 0.0;
        for t in (0..500).step_by(10) {
            output = estimator.update(0, t as u32);
        }
        assert!(output.abs() < EPS, "expected ~0 output RPM, got {output}");
    }

    const EPS: f64 = 1e-9;
}
