//! Position/time sampling for the velocity estimator.
//!
//! The [`VelocityEstimator`](crate::velocity_estimator::VelocityEstimator)
//! differentiates raw encoder ticks against the Brain's clock. This module
//! defines the [`TimestampedPosition`] source that feeds it and implements it for
//! a V5 Smart [`Motor`].

use std::time::Duration;

use vexide::{
    math::Direction,
    prelude::sleep,
    smart::{motor::Motor, PortError, SmartDevice},
    time::LowResolutionTime,
};

/// Whether `Motor::raw_position()` applies the motor's configured
/// [`Direction`](vexide::smart::motor::Direction) — i.e. reports the same sign as
/// `Motor::position()`.
///
/// If `false`, callers must negate the estimator's output for motors configured
/// [`Direction::Reverse`](vexide::smart::motor::Direction::Reverse).
// TODO: verify on hardware with `probe_direction`.
pub const MOTOR_RAW_POSITION_RESPECTS_DIRECTION: bool = true;

/// A source of a device's raw encoder position tagged with the Brain's clock
/// reading.
pub trait TimestampedPosition {
    type Error;
    /// Returns (raw encoder ticks, the Brain's clock reading in milliseconds).
    fn timestamped_position(&self) -> Result<(i32, u32), Self::Error>;
}

impl TimestampedPosition for Motor {
    type Error = PortError;

    fn timestamped_position(&self) -> Result<(i32, u32), Self::Error> {
        let ticks = self.raw_position()?;

        // `Motor::timestamp()` and the old `vexDeviceMotorPositionRawGet` out-param
        // return the same value: `vexSystemTimeGet()` sampled when CPU1's V5_Device
        // simpletask published this motor's packet. V5 motors transmit no timestamp of
        // their own (their data packet carries only temperature, current, position,
        // velocity, voltage, flags, faults), so the motor's actual sample time is not
        // observable and may precede this by up to 10 ms. Both values refresh only
        // during `vexTasksRun`, so these two reads always describe the same packet.
        let timestamp = self
            .timestamp()?
            .duration_since(LowResolutionTime::EPOCH)
            .as_millis() as u32;

        Ok((ticks, timestamp))
    }
}

/// One-shot hardware diagnostic for [`MOTOR_RAW_POSITION_RESPECTS_DIRECTION`] and
/// `TICKS_PER_INTERNAL_REV`.
///
/// Configures `motor` as [`Direction::Reverse`] — required, since under
/// [`Direction::Forward`] `raw_position()` and `position()` move the same way
/// regardless of whether the raw reading honors the flag — drives it at a low
/// positive voltage for ~500 ms, then compares the raw tick change against
/// `position()`, which definitively applies the flag. It prints whether the two
/// agree (the value the constant should hold) and the measured ticks per internal
/// revolution (expect ~50). The original direction is always restored before
/// returning. Run once against a free-spinning motor; nothing in the normal code
/// path calls this.
pub async fn probe_direction(motor: &mut Motor) -> Result<(), PortError> {
    let original = motor.direction()?;
    motor.set_direction(Direction::Reverse)?;

    // Measure with the flag set, then *always* restore the original direction —
    // never leave the motor reconfigured, on any path.
    let measured = drive_and_measure(motor).await;
    motor.set_direction(original)?;
    let (raw_delta, pos_delta) = measured?;

    // No motion on either reading means the motor stalled or is disconnected:
    // nothing to compare, and `pos_delta == 0` would divide to inf/NaN below.
    if raw_delta == 0 || pos_delta == 0.0 {
        println!(
            "probe_direction: no motion over 500 ms at +3 V (raw_delta = {raw_delta} ticks, \
             pos_delta = {pos_delta:.4} rev; motor stalled or disconnected?) — inconclusive."
        );
        return Ok(());
    }

    // `position()` applies the direction flag; `raw_position()` honors it only if
    // the two move the same way under Reverse.
    let respects_direction = (raw_delta > 0) == (pos_delta > 0.0);

    // Ticks per revolution of the 3600 RPM internal rotor: raw ticks per *output*
    // revolution divided by the gearset reduction (blue = 6.0).
    let gearset_ratio = 3600.0 / motor.gearset()?.max_rpm();
    let ticks_per_internal_rev = (raw_delta as f64 / pos_delta).abs() / gearset_ratio;

    println!(
        "probe_direction: raw_delta = {raw_delta} ticks, pos_delta = {pos_delta:.4} rev \
         over 500 ms at +3 V (Reverse). Set MOTOR_RAW_POSITION_RESPECTS_DIRECTION = \
         {respects_direction}; TICKS_PER_INTERNAL_REV ≈ {ticks_per_internal_rev:.1} (expect ~50)."
    );
    Ok(())
}

/// Drives `motor` at +3 V for 500 ms and returns `(raw tick delta, output-shaft
/// revolution delta)`. Always stops the motor before returning, even if a read
/// fails, so the caller only has to restore the direction flag.
async fn drive_and_measure(motor: &mut Motor) -> Result<(i32, f64), PortError> {
    let raw_start = motor.raw_position()?;
    let pos_start = motor.position()?;

    motor.set_voltage(3.0)?;
    sleep(Duration::from_millis(500)).await;

    // Read before stopping, but stop regardless of whether the reads succeeded.
    let raw_end = motor.raw_position();
    let pos_end = motor.position();
    let _ = motor.set_voltage(0.0);

    let raw_delta = raw_end? - raw_start;
    let pos_delta = (pos_end? - pos_start).as_turns();
    Ok((raw_delta, pos_delta))
}
