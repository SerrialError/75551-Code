//! Position/time sampling for the velocity estimator.
//!
//! The [`VelocityEstimator`](crate::velocity_estimator::VelocityEstimator)
//! differentiates raw encoder ticks against the Brain's clock. This module
//! defines the [`TimestampedPosition`] source that feeds it and implements it for
//! a V5 Smart [`Motor`].

use vexide::{
    smart::{motor::Motor, PortError, SmartDevice},
    time::LowResolutionTime,
};

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
