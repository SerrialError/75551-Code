//! Position/time sampling for the velocity estimator.
//!
//! The [`VelocityEstimator`](crate::velocity_estimator::VelocityEstimator)
//! differentiates raw encoder ticks against the device's own clock. This module
//! defines the [`TimestampedPosition`] source that feeds it and implements it for
//! a V5 Smart [`Motor`].

use vexide::{
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
