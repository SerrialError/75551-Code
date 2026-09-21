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
//! # Where the feedback lives
//!
//! All of it is in the drivetrain's inner per-side velocity loop, none of it
//! here. Each side's [`MotorFeedforward`] turns the commanded velocity and
//! acceleration into volts, and each side's optional velocity [`Pid`] trims what
//! the feedforward gets wrong, both configured on
//! [`VelocityDifferentialConfig`]. This module adds no loop of its own: it reads
//! the clock, interpolates the profile, and hands the setpoints over.
//!
//! That makes a run **open loop in position**. The velocity loop can hold each
//! wheel at its commanded speed, but nothing measures where the robot ended up,
//! so wheel slip, a scrubbed turn, or a wheel diameter that is off by a percent
//! all integrate into position error that never gets pulled back. Accuracy rests
//! entirely on the feedforward constants and the velocity loop tracking well.
//! Check the endpoint against what the profile promised before trusting a path,
//! and expect error to grow with path length.
//!
//! # Seeing what the robot actually did
//!
//! [`follow_logged`] replays a profile exactly as [`follow`] does, but samples
//! the tracking system's robot-frame velocity every tick and prints the run as
//! Desmos list literals when it ends:
//!
//! ```text
//! L=[(t,linear velocity)..]    # m/s
//! A=[(t,angular velocity)..]   # rad/s, or m/s^2 when nothing measures rotation
//! ```
//!
//! Paste those against the profile's own `linear_velocity` and
//! `angular_velocity` to see where the velocity loop fell behind. Since the run
//! is open loop in position, that gap is the whole story of where the robot
//! ended up.
//!
//! [`MotorFeedforward`]: evian::control::loops::MotorFeedforward
//! [`Pid`]: evian::control::loops::Pid
//! [`VelocityDifferentialConfig`]: crate::velocity_differential::VelocityDifferentialConfig
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

use std::{
    fmt::Write,
    time::{Duration, Instant},
};

use evian::{
    drivetrain::Drivetrain,
    tracking::{Tracking, TracksVelocity},
};
use vexide::prelude::{sleep, Motor};

use crate::{
    desmos,
    velocity_differential::{TankVelocity, WheelSetpoint},
};

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

/// Unit conversion and loop rate for a [`follow`] run.
///
/// There are no controllers here. The only feedback in the path is the
/// drivetrain's own per-side velocity loop; see the module docs.
pub struct ProfileConfig {
    /// Wheel units per metre, converting vmplib's SI lengths into the units the
    /// drivetrain is configured in. [`INCHES_PER_METER`] for inches, `1.0` for a
    /// robot measured in metres.
    pub wheel_units_per_meter: f64,
    /// How long to sleep between setpoint updates.
    pub update_interval: Duration,
}

impl Default for ProfileConfig {
    /// Inches, updated every [`Motor::WRITE_INTERVAL`].
    fn default() -> Self {
        Self {
            wheel_units_per_meter: INCHES_PER_METER,
            update_interval: Motor::WRITE_INTERVAL,
        }
    }
}

/// Drives `samples` in real time, returning when the profile's last timestamp
/// elapses.
///
/// Both sides are commanded to zero before returning, on every path. Port errors
/// don't abort the run: a single dropped packet shouldn't strand the robot
/// mid-path, so the last error is returned once the profile finishes instead.
///
/// An empty `samples` is a no-op.
pub async fn follow<M, T>(
    drivetrain: &mut Drivetrain<M, T>,
    samples: &[DriveSample],
    config: &ProfileConfig,
) -> Result<(), M::Error>
where
    M: TankVelocity,
    T: Tracking,
{
    follow_with(drivetrain, samples, config, |_, _: &T| {}).await
}

/// The same replay as [`follow`], with the robot's measured velocity sampled
/// once per control tick and printed as Desmos lists when the profile finishes.
///
/// Nothing about the driving changes: the measurements are read from the
/// tracking system and never fed back, so a logged run commands the motors
/// exactly as a plain one does. This is the only way to see what the robot
/// actually did against what the profile asked for, since the replay is open
/// loop in position and never corrects itself.
///
/// Printing happens once the profile ends and both sides are stopped. Writing to
/// the console mid-replay would stall the loop and skew the very timing being
/// measured.
///
/// See [`desmos_blocks`] for the output's format and units.
pub async fn follow_logged<M, T>(
    drivetrain: &mut Drivetrain<M, T>,
    samples: &[DriveSample],
    config: &ProfileConfig,
) -> Result<(), M::Error>
where
    M: TankVelocity,
    T: TracksVelocity,
{
    let mut log = Vec::new();
    let result = follow_with(drivetrain, samples, config, |time, tracking: &T| {
        log.push(Measurement {
            time,
            linear_velocity: tracking.linear_velocity(),
            angular_velocity: tracking.angular_velocity(),
        });
    })
    .await;

    print!("{}", desmos_blocks(&log, config.wheel_units_per_meter));

    result
}

/// The replay loop behind [`follow`] and [`follow_logged`], calling `on_tick`
/// with the elapsed profile time and the tracking system once per iteration.
async fn follow_with<M, T, F>(
    drivetrain: &mut Drivetrain<M, T>,
    samples: &[DriveSample],
    config: &ProfileConfig,
    mut on_tick: F,
) -> Result<(), M::Error>
where
    M: TankVelocity,
    T: Tracking,
    F: FnMut(f64, &T),
{
    let Some(last) = samples.last() else {
        return Ok(());
    };

    let scale = config.wheel_units_per_meter;
    let start = Instant::now();
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

        // Sampled before this tick's command lands, so a measurement pairs with
        // the state the robot was actually in at `elapsed`.
        on_tick(elapsed, &drivetrain.tracking);

        let left = WheelSetpoint {
            velocity: sample.left_velocity * scale,
            acceleration: sample.left_accel * scale,
        };
        let right = WheelSetpoint {
            velocity: sample.right_velocity * scale,
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

/// One logged instant of a replay: seconds since the profile started, and the
/// robot-frame velocities the tracking system reported at that moment.
///
/// Linear velocity is in wheel units per second, whatever the drivetrain is
/// configured in; angular velocity is in radians per second, counterclockwise
/// positive.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Measurement {
    time: f64,
    linear_velocity: f64,
    angular_velocity: f64,
}

/// A replay's measurements as Desmos list literals: `L` of `(t, linear
/// velocity)` points, then `A` of either `(t, angular velocity)` or
/// `(t, linear acceleration)` points.
///
/// `A` carries the measured angular velocity whenever the tracking system
/// reported any rotation at all. A tracking system with no heading source
/// reports a flat zero, which plots as nothing worth looking at; in that case
/// `A` carries the robot's linear acceleration instead, central-differenced from
/// the measured velocities. The comment line above each list says which of the
/// two it holds.
///
/// Values are converted back into the profile's own units — metres, radians,
/// seconds — so the lists overlay the profile that was being followed rather
/// than sitting a wheel-unit conversion away from it.
///
/// Each list is alone on its own line: paste one line at a time, and leave the
/// `#` comment lines behind, since Desmos won't take them.
fn desmos_blocks(log: &[Measurement], wheel_units_per_meter: f64) -> String {
    // An unset (zero) scale would turn every measurement into an infinity Desmos
    // can't read, so fall back to reporting raw wheel units.
    let scale = if wheel_units_per_meter > 0.0 {
        wheel_units_per_meter
    } else {
        1.0
    };

    let mut out = String::from("\n# measured motion profile response\n");
    out.push_str("# L = measured linear velocity (m/s)\n");
    let _ = writeln!(
        out,
        "L={}",
        desmos::points(log.iter().map(|m| (m.time, m.linear_velocity / scale)))
    );

    if log.iter().any(|m| m.angular_velocity != 0.0) {
        out.push_str("# A = measured angular velocity (rad/s)\n");
        let _ = writeln!(
            out,
            "A={}",
            desmos::points(log.iter().map(|m| (m.time, m.angular_velocity)))
        );
    } else {
        out.push_str("# A = measured linear acceleration (m/s^2); tracking reported no rotation\n");
        let _ = writeln!(
            out,
            "A={}",
            desmos::points(
                accelerations(log)
                    .into_iter()
                    .map(|(time, accel)| (time, accel / scale))
            )
        );
    }

    out
}

/// `(t, acceleration)` at every measurement, central-differenced from the
/// measured linear velocities so the points keep `L`'s timestamps. The first and
/// last are one-sided differences, and a lone measurement has no slope to take,
/// so it reads zero.
fn accelerations(log: &[Measurement]) -> Vec<(f64, f64)> {
    (0..log.len())
        .map(|index| {
            let before = &log[index.saturating_sub(1)];
            let after = &log[(index + 1).min(log.len() - 1)];
            let span = after.time - before.time;
            let accel = if span > 0.0 {
                (after.linear_velocity - before.linear_velocity) / span
            } else {
                0.0
            };
            (log[index].time, accel)
        })
        .collect()
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
    use super::{accelerations, desmos_blocks, sample_at, DriveSample, Measurement};

    fn measurement(time: f64, linear: f64, angular: f64) -> Measurement {
        Measurement {
            time,
            linear_velocity: linear,
            angular_velocity: angular,
        }
    }

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

    #[test]
    fn logs_velocities_as_desmos_point_lists() {
        let log = [measurement(0.0, 39.370_078_74, 0.5), measurement(0.01, 0.0, -0.25)];
        let blocks = desmos_blocks(&log, super::INCHES_PER_METER);

        // Wheel units (inches here) come back out in the profile's metres.
        assert!(blocks.contains("\nL=[(0.0000,1.0000),(0.0100,0.0000)]\n"));
        assert!(blocks.contains("\nA=[(0.0000,0.5000),(0.0100,-0.2500)]\n"));
    }

    #[test]
    fn falls_back_to_acceleration_without_angular_velocity() {
        // 1 m/s^2 in wheel units, with the tracking system reporting no rotation.
        let scale = super::INCHES_PER_METER;
        let log = [
            measurement(0.0, 0.0, 0.0),
            measurement(1.0, scale, 0.0),
            measurement(2.0, 2.0 * scale, 0.0),
        ];
        let blocks = desmos_blocks(&log, scale);

        assert!(blocks.contains("# A = measured linear acceleration"));
        assert!(blocks.contains("\nA=[(0.0000,1.0000),(1.0000,1.0000),(2.0000,1.0000)]\n"));
    }

    #[test]
    fn a_single_measurement_has_no_slope() {
        assert_eq!(accelerations(&[measurement(0.5, 2.0, 0.0)]), [(0.5, 0.0)]);
    }

    #[test]
    fn an_empty_log_still_prints_both_lists() {
        let blocks = desmos_blocks(&[], super::INCHES_PER_METER);
        assert!(blocks.contains("\nL=[]\n"));
        assert!(blocks.contains("\nA=[]\n"));
    }

    #[test]
    fn an_unset_scale_reports_wheel_units_instead_of_infinity() {
        let blocks = desmos_blocks(&[measurement(0.0, 3.0, 1.0)], 0.0);
        assert!(blocks.contains("\nL=[(0.0000,3.0000)]\n"));
    }
}
