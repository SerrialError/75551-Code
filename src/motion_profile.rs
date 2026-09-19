//! Replaying a pre-generated drive motion profile.
//!
//! A *motion profile* here is a dense time series of drivetrain setpoints
//! computed offline by vmplib. Each [`DriveSample`] holds the robot-frame linear
//! and angular velocity together with the per-side wheel velocity and
//! acceleration at one instant, in SI units (metres, radians, seconds).
//! [`follow`] replays that series in real time.
//!
//! # What the follower actually commands
//!
//! The profile's `left_velocity` / `right_velocity` go straight to the
//! drivetrain as per-side setpoints through
//! [`TankVelocity`](crate::velocity_differential::TankVelocity), so the arcade
//! mixing in [`drive_arcade`](evian::drivetrain::model::Arcade::drive_arcade) is
//! bypassed entirely. That matters for two reasons. The profile already solved
//! the inverse kinematics, so re-deriving them from `linear_velocity` and
//! `angular_velocity` would only add rounding and a track-width mismatch. And it
//! lets `left_accel` / `right_accel` reach the feedforward's `ka` term as exact
//! values instead of a finite difference of the velocity targets, which is the
//! whole reason a profile carries acceleration in the first place.
//!
//! # Correction
//!
//! Replaying velocities open loop is a dead reckoning bet: every unmodelled
//! loss, every wheel slip, every volt of battery sag integrates straight into
//! position error, and nothing ever pulls it back. So the follower integrates
//! the profile's own `linear_velocity` and `angular_velocity` into a *reference*
//! forward travel and heading, then runs two correction loops against what the
//! tracking system reports. The linear correction is added to both sides, the
//! angular correction is added to one and subtracted from the other.
//!
//! Both controllers are optional. Leave them `None` for a pure open-loop replay,
//! which is worth doing once when first checking a profile: if open loop already
//! lands close, the feedforward constants are good and the correction gains only
//! have to clean up the remainder.
//!
//! There is no cross-track correction, because these samples carry no pose. A
//! profile that has drifted sideways is corrected back to the right *heading* and
//! the right *distance along the path*, not to the path itself.
//!
//! # Units
//!
//! vmplib emits metres; evian works in "wheel units", whatever the wheel
//! diameter was measured in. [`ProfileConfig::wheel_units_per_meter`] bridges the
//! two. Use [`INCHES_PER_METER`] if the rest of the robot is configured in
//! inches, which is the convention the rest of this project uses.
//!
//! Angles need no conversion: vmplib and evian both use radians, both
//! counterclockwise-positive.

use std::time::{Duration, Instant};

use evian::{
    control::loops::Feedback,
    drivetrain::Drivetrain,
    math::Angle,
    tracking::{TracksForwardTravel, TracksHeading},
};
use vexide::prelude::sleep;

use crate::velocity_differential::{TankVelocity, WheelSetpoint};

/// Wheel units per metre for a robot measured in inches.
pub const INCHES_PER_METER: f64 = 39.370_078_740_157_48;

/// One timestep of a vmplib drive motion profile.
///
/// All fields are SI: metres, radians, seconds. `angular_velocity` is
/// counterclockwise-positive, and `left_velocity` / `right_velocity` are the
/// linear velocities of the left and right wheels, not angular.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DriveSample {
    /// Seconds since the start of the profile.
    pub time: f64,
    /// Linear velocity at the centre of the robot, in m/s.
    pub linear_velocity: f64,
    /// Angular velocity, in rad/s, counterclockwise positive.
    pub angular_velocity: f64,
    /// Left wheel linear velocity, in m/s.
    pub left_velocity: f64,
    /// Right wheel linear velocity, in m/s.
    pub right_velocity: f64,
    /// Left wheel linear acceleration, in m/s^2.
    pub left_accel: f64,
    /// Right wheel linear acceleration, in m/s^2.
    pub right_accel: f64,
}

/// Unit conversion, correction loops, and loop rate for a [`follow`] run.
///
/// `L` corrects forward travel (wheel units in, wheel units / second out) and
/// `A` corrects heading (an [`Angle`] in, radians / second out). Either may be
/// `None`, in which case that axis is left uncorrected.
pub struct ProfileConfig<L, A> {
    /// Corrects the robot's forward travel against the profile's integrated
    /// `linear_velocity`. Output is added to both sides, in wheel units /
    /// second.
    pub linear_controller: Option<L>,
    /// Corrects the robot's heading against the profile's integrated
    /// `angular_velocity`. Output is in radians / second, counterclockwise
    /// positive.
    pub angular_controller: Option<A>,
    /// Wheel units per metre, converting vmplib's SI lengths into the units the
    /// drivetrain is configured in. [`INCHES_PER_METER`] for inches, `1.0` for a
    /// robot measured in metres.
    pub wheel_units_per_meter: f64,
    /// How long to sleep between setpoint updates.
    pub update_interval: Duration,
}

/// Drives `samples` in real time, returning when the profile's last timestamp
/// elapses.
///
/// Both sides are commanded to zero before returning, on every path. Port errors
/// don't abort the run: a single dropped packet shouldn't strand the robot
/// mid-path, so the last error is returned once the profile finishes instead.
///
/// An empty `samples` is a no-op.
pub async fn follow<M, T, L, A>(
    drivetrain: &mut Drivetrain<M, T>,
    samples: &[DriveSample],
    config: &mut ProfileConfig<L, A>,
) -> Result<(), M::Error>
where
    M: TankVelocity,
    T: TracksForwardTravel + TracksHeading,
    L: Feedback<State = f64, Signal = f64>,
    A: Feedback<State = Angle, Signal = f64>,
{
    let (Some(first), Some(last)) = (samples.first(), samples.last()) else {
        return Ok(());
    };

    let scale = config.wheel_units_per_meter;
    let half_track = drivetrain.model.track_width() / 2.0;

    // The samples describe velocities, not a pose, so the reference they imply
    // is their running integral from wherever the robot currently sits. Anchor
    // on the tracking system's readings now rather than assuming it was zeroed.
    let initial_travel = drivetrain.tracking.forward_travel();
    let initial_heading = drivetrain.tracking.heading();
    let mut reference_travel = 0.0;
    let mut reference_heading = 0.0;

    let start = Instant::now();
    let mut prev_time = start;
    let mut prev_linear = first.linear_velocity * scale;
    let mut prev_angular = first.angular_velocity;
    let mut result = Ok(());

    loop {
        let elapsed = start.elapsed().as_secs_f64();
        if elapsed >= last.time {
            break;
        }

        // Interpolating at wall time, rather than stepping an index once per
        // iteration, keeps the replay on the profile's clock even when a loop
        // iteration runs long.
        let sample = sample_at(samples, elapsed);

        let dt = prev_time.elapsed();
        prev_time = Instant::now();
        let dt_secs = dt.as_secs_f64();

        // Trapezoidal integration of the commanded velocities. This uses the
        // loop's own dt, not the profile timestep, because the sample was
        // interpolated at wall time and the two would otherwise drift apart.
        let linear = sample.linear_velocity * scale;
        let angular = sample.angular_velocity;
        reference_travel += 0.5 * (prev_linear + linear) * dt_secs;
        reference_heading += 0.5 * (prev_angular + angular) * dt_secs;
        prev_linear = linear;
        prev_angular = angular;

        let travel_correction = match config.linear_controller.as_mut() {
            Some(controller) => controller.update(
                drivetrain.tracking.forward_travel(),
                initial_travel + reference_travel,
                dt,
            ),
            None => 0.0,
        };
        let heading_correction = match config.angular_controller.as_mut() {
            Some(controller) => controller.update(
                drivetrain.tracking.heading(),
                initial_heading + Angle::from_radians(reference_heading),
                dt,
            ),
            None => 0.0,
        };

        // Counterclockwise-positive, matching the profile's own sign convention:
        // turning left means the right wheels speed up and the left slow down.
        let left = WheelSetpoint {
            velocity: sample.left_velocity * scale + travel_correction
                - heading_correction * half_track,
            acceleration: sample.left_accel * scale,
        };
        let right = WheelSetpoint {
            velocity: sample.right_velocity * scale
                + travel_correction
                + heading_correction * half_track,
            acceleration: sample.right_accel * scale,
        };

        if let Err(err) = drivetrain.model.drive_tank_velocity(left, right) {
            result = Err(err);
        }

        sleep(config.update_interval).await;
    }

    if let Err(err) = drivetrain
        .model
        .drive_tank_velocity(WheelSetpoint::default(), WheelSetpoint::default())
    {
        result = Err(err);
    }

    result
}

/// The profile linearly interpolated at `time` seconds, clamped to the first and
/// last sample outside the profile's span.
///
/// `samples` must be non-empty and sorted by `time`; [`follow`] guarantees the
/// first and vmplib guarantees the second.
fn sample_at(samples: &[DriveSample], time: f64) -> DriveSample {
    // Index of the first sample *after* `time`, so the pair to interpolate
    // between is `next - 1` and `next`.
    let next = samples.partition_point(|sample| sample.time <= time);
    if next == 0 {
        return samples[0];
    }
    let (Some(before), Some(after)) = (samples.get(next - 1), samples.get(next)) else {
        return samples[samples.len() - 1];
    };

    // Two samples sharing a timestamp would divide by zero; hold the earlier one.
    let span = after.time - before.time;
    let t = if span > 0.0 {
        (time - before.time) / span
    } else {
        0.0
    };

    DriveSample {
        time,
        linear_velocity: lerp(before.linear_velocity, after.linear_velocity, t),
        angular_velocity: lerp(before.angular_velocity, after.angular_velocity, t),
        left_velocity: lerp(before.left_velocity, after.left_velocity, t),
        right_velocity: lerp(before.right_velocity, after.right_velocity, t),
        left_accel: lerp(before.left_accel, after.left_accel, t),
        right_accel: lerp(before.right_accel, after.right_accel, t),
    }
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

#[cfg(test)]
mod tests {
    use super::{sample_at, DriveSample};

    fn sample(time: f64, left: f64, right: f64) -> DriveSample {
        DriveSample {
            time,
            left_velocity: left,
            right_velocity: right,
            ..DriveSample::default()
        }
    }

    #[test]
    fn interpolates_between_samples() {
        let samples = [sample(0.0, 0.0, 1.0), sample(0.1, 1.0, 2.0)];
        let mid = sample_at(&samples, 0.025);
        assert!((mid.left_velocity - 0.25).abs() < 1e-12);
        assert!((mid.right_velocity - 1.25).abs() < 1e-12);
    }

    #[test]
    fn clamps_outside_the_profile() {
        let samples = [sample(1.0, 3.0, 4.0), sample(2.0, 5.0, 6.0)];
        assert_eq!(sample_at(&samples, 0.0).left_velocity, 3.0);
        assert_eq!(sample_at(&samples, 9.0).right_velocity, 6.0);
    }

    #[test]
    fn lands_on_exact_timestamps() {
        let samples = [
            sample(0.0, 0.0, 0.0),
            sample(0.1, 1.0, 2.0),
            sample(0.2, 9.0, 9.0),
        ];
        let hit = sample_at(&samples, 0.1);
        assert_eq!(hit.left_velocity, 1.0);
        assert_eq!(hit.right_velocity, 2.0);
    }
}
