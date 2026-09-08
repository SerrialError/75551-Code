//! Position/time sampling for the velocity estimator.
//!
//! The [`VelocityEstimator`](crate::velocity_estimator::VelocityEstimator)
//! differentiates raw encoder ticks against the device's own clock. This module
//! defines the [`TimestampedPosition`] source that feeds it and implements it for
//! a V5 Smart [`Motor`].

use std::time::Duration;

use vexide::{
    math::Direction,
    prelude::sleep,
    smart::{motor::Motor, PortError, SmartDevice},
    time::LowResolutionTime,
};

/// Whether `Motor::raw_position()` already accounts for the motor's configured
/// [`Direction`](vexide::smart::motor::Direction) (so a reversed motor reads
/// negative ticks when driven "forward").
///
/// If this is `false`, callers must negate the estimator's output for motors
/// configured [`Direction::Reverse`](vexide::smart::motor::Direction::Reverse).
// TODO: verify on hardware.
pub const MOTOR_RAW_POSITION_RESPECTS_DIRECTION: bool = true;

/// A source of a device's raw encoder position tagged with the device's own
/// clock reading, both sampled as close together as the API allows.
pub trait TimestampedPosition {
    type Error;
    /// Returns (raw encoder ticks, device clock reading in milliseconds).
    fn timestamped_position(&self) -> Result<(i32, u32), Self::Error>;
}

impl TimestampedPosition for Motor {
    type Error = PortError;

    fn timestamped_position(&self) -> Result<(i32, u32), Self::Error> {
        let ticks = self.raw_position()?;

        // TODO: this is the Brain's packet-processed timestamp, not the motor's own record
        // of when it sampled, and the two reads below may describe different samples. Swap
        // to the vexDeviceMotorPositionRawGet out-param once vexide exposes it (vexide#386).
        let timestamp = self
            .timestamp()?
            .duration_since(LowResolutionTime::EPOCH)
            .as_millis() as u32;

        Ok((ticks, timestamp))
    }
}

/// One-shot hardware diagnostic for [`MOTOR_RAW_POSITION_RESPECTS_DIRECTION`].
///
/// Configures `motor` as [`Direction::Reverse`], drives it at a low positive
/// voltage for ~500 ms, and prints the sign of the resulting change in
/// `raw_position()` along with the value the constant should hold. Leaves the
/// motor stopped. Run once against a free-spinning motor and set the constant to
/// match; nothing in the normal code path calls this.
// One-shot diagnostic, wired up by hand when characterizing hardware.
#[allow(dead_code)]
pub async fn probe_direction(motor: &mut Motor) {
    let _ = motor.set_direction(Direction::Reverse);

    let start = motor.raw_position().unwrap_or(0);
    let _ = motor.set_voltage(3.0);
    sleep(Duration::from_millis(500)).await;
    let end = motor.raw_position().unwrap_or(start);
    let _ = motor.set_voltage(0.0);

    // A Reverse-configured motor driven at *positive* voltage spins physically
    // backward. If raw_position() honors the direction flag its reported ticks go
    // negative; if it reports the bare encoder they go positive.
    let delta = end - start;
    if delta == 0 {
        println!(
            "probe_direction: raw_position() did not change over 500 ms at +3 V \
             (motor stalled or disconnected?) — inconclusive."
        );
        return;
    }
    let respects_direction = delta < 0;
    println!(
        "probe_direction: raw_position() delta = {delta} over 500 ms at +3 V (Reverse). \
         Set MOTOR_RAW_POSITION_RESPECTS_DIRECTION = {respects_direction}."
    );
}
