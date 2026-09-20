//! A differential drivetrain model with a cascaded velocity controller.
//!
//! Where evian's stock `Differential` interprets its inputs as raw normalized
//! voltages, [`VelocityDifferential`] treats the motion commands' outputs as
//! *velocities* and runs an inner per-side loop that turns them into voltages:
//!
//! ```text
//! left_v  = linear_v - angular_v * (track_width / 2)   [in/s]
//! right_v = linear_v + angular_v * (track_width / 2)   [in/s]
//! target_w = side_v / wheel_radius                     [rad/s]
//! volts    = feedforward(target_w, target_a) + feedback(target_w - measured_w)
//! ```
//!
//! Each side's feedforward (`FF`) and feedback (`FB`) are independently optional;
//! a missing half contributes `0.0` volts. The velocity feedback comes from a
//! per-side [`WheelVelocity`] source (`S`); the built-in [`MotorGroupVelocity`]
//! averages the drive motors' filtered velocities published by a
//! [`MotorVelocityTracker`], since `WheeledTracking` only reports robot-frame
//! velocity.
//!
//! [`TankVelocity`] is the second way in. It skips the arcade mixing and takes
//! per-side velocity and acceleration setpoints directly, which is what
//! [`motion_profile::follow`](crate::motion_profile::follow) needs to hand the
//! feedforward a profile's own acceleration instead of a finite difference.
//!
//! # Which way is positive
//!
//! `steer` is **counterclockwise-positive**: a positive value speeds the right
//! wheels up and slows the left, turning the robot left. This matches evian's
//! own frame, where [`TracksHeading`] documents anticlockwise as a positive
//! rotation and [`TracksVelocity::angular_velocity`] reports in that frame, and
//! it matches the SI convention a vmplib profile's `angular_velocity` uses.
//!
//! evian itself is not consistent about this, so it is worth knowing which of
//! its motions agree. `Basic::drive_distance`, `Basic::drive_distance_at_heading`,
//! `Basic::turn_to_heading`, and `Seeking::move_to_point` all sign their angular
//! output counterclockwise-positive and work correctly here.
//! `Basic::turn_to_point` and `Seeking::boomerang` sign theirs the other way
//! (they feed `AngularPid` a negated error against a zero setpoint, which lands
//! on `heading - target` rather than `target - heading`) and will turn away from
//! their target with this model. No convention satisfies both halves; this one
//! satisfies the motions `main.rs` actually calls, plus the tracking system, plus
//! the profile format. Fixing the other two belongs upstream in the evian fork.
//!
//! Note that evian's blanket `impl<T: Tank> Arcade for T` is clockwise-positive,
//! so a drivetrain built on the stock `Differential` behaves the opposite way.
//! Teleop code that scales a stick's x-axis into `steer` needs a negation, since
//! a stick pushed right asks for a clockwise turn.
//!
//! [`TracksHeading`]: evian::tracking::TracksHeading
//! [`TracksVelocity::angular_velocity`]: evian::tracking::TracksVelocity::angular_velocity
//!
//! The inner loop regulates each wheel's angular velocity in **radians / second**
//! — hence a plain `Pid` over `f64` rather than an `AngularPid`, whose `±π` error
//! wrapping is correct for a heading but nonsense for a velocity. `throttle` is in
//! inches / second, `steer` in radians / second, and lengths in inches.

use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use evian::{
    control::loops::{Feedback, Feedforward, MotorFeedforwardSetpoint},
    drivetrain::model::{Arcade, DrivetrainModel},
    math::desaturate,
};
use vexide::{
    prelude::Motor,
    smart::{motor::Gearset, PortError},
};

use crate::motor_velocity::{live_rpm_sum, wheel_omega_from_rpm, MotorVelocityTracker};

/// A source of a drivetrain side's measured wheel angular velocity, in
/// radians / second.
pub trait WheelVelocity {
    fn velocity(&mut self) -> f64;
}

/// One side's wheel velocity setpoint and the acceleration the feedforward's
/// `ka` term should assume, in inches / second and inches / second^2.
///
/// These are *linear* wheel velocities, the same units as `drive_arcade`'s
/// `throttle`; the model converts them to radians / second internally.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WheelSetpoint {
    pub velocity: f64,
    pub acceleration: f64,
}

/// A differential model that takes per-side wheel setpoints directly, skipping
/// arcade mixing.
///
/// [`Arcade`] can only express a velocity, so its model has to finite-difference
/// the acceleration that feeds the feedforward's `ka` term. A caller replaying a
/// motion profile already has the exact per-side acceleration and should not
/// have it thrown away and re-estimated. Such a caller also usually has exact
/// per-side velocities, which arcade mixing would round-trip through a
/// track-width that may not match the one the profile was generated against.
pub trait TankVelocity: DrivetrainModel {
    /// Commands both sides. `left` and `right` are in inches / second and
    /// inches / second^2.
    fn drive_tank_velocity(
        &mut self,
        left: WheelSetpoint,
        right: WheelSetpoint,
    ) -> Result<(), Self::Error>;
}

/// A [`WheelVelocity`] source backed by a [`MotorVelocityTracker`], averaging the
/// group's filtered output-shaft velocities.
pub struct MotorGroupVelocity {
    /// Owns the background tracker so its task lives exactly as long as this
    /// source (and the drivetrain that holds it).
    tracker: MotorVelocityTracker,
    /// Wheel revolutions per motor output-shaft revolution (external gearing
    /// only; the tracker already reports gearset-reduced RPM). `1.0` for direct
    /// drive.
    gear_ratio: f64,
}

impl MotorGroupVelocity {
    pub fn new(tracker: MotorVelocityTracker, gear_ratio: f64) -> Self {
        Self {
            tracker,
            gear_ratio,
        }
    }
}

impl WheelVelocity for MotorGroupVelocity {
    fn velocity(&mut self) -> f64 {
        let gear_ratio = self.gear_ratio;
        self.tracker.with_velocities(|velocities| {
            let (sum_rpm, count) = live_rpm_sum(velocities);
            if count == 0 {
                return 0.0;
            }
            wheel_omega_from_rpm(sum_rpm / count as f64, gear_ratio)
        })
    }
}

/// Tuning constants and optional per-side controllers for a
/// [`VelocityDifferential`].
pub struct VelocityDifferentialConfig<FF, FB> {
    pub left_velocity_feedforward: Option<FF>,
    pub right_velocity_feedforward: Option<FF>,
    pub left_velocity_feedback: Option<FB>,
    pub right_velocity_feedback: Option<FB>,
    /// Wheel diameter, in inches.
    pub wheel_diameter: f64,
    /// Distance between the left and right wheels, in inches.
    pub track_width: f64,
    /// Maximum wheel angular velocity (rad/s) used to desaturate turns; `0.0`
    /// disables desaturation.
    pub max_velocity: f64,
}

/// A differential drivetrain driven by a cascaded velocity controller.
pub struct VelocityDifferential<FF, FB, S> {
    left: Rc<RefCell<dyn AsMut<[Motor]>>>,
    right: Rc<RefCell<dyn AsMut<[Motor]>>>,

    left_source: S,
    right_source: S,

    config: VelocityDifferentialConfig<FF, FB>,

    prev_time: Option<Instant>,
    prev_left_target: f64,
    prev_right_target: f64,
}

impl<FF, FB> VelocityDifferential<FF, FB, MotorGroupVelocity> {
    /// Reads its velocity feedback from the drive motors' own encoders.
    /// `gear_ratio` configures the built-in [`MotorGroupVelocity`] sources and
    /// `gearset` seeds their estimators (shared across every motor); for a
    /// custom velocity source, use [`with_sources`](Self::with_sources) instead.
    pub fn new(
        left: Rc<RefCell<dyn AsMut<[Motor]>>>,
        right: Rc<RefCell<dyn AsMut<[Motor]>>>,
        gear_ratio: f64,
        gearset: Gearset,
        config: VelocityDifferentialConfig<FF, FB>,
    ) -> Self {
        // Each side gets its own background estimator, owned by its source so the
        // task runs exactly as long as the drivetrain holds the source.
        let left_source =
            MotorGroupVelocity::new(MotorVelocityTracker::new(left.clone(), gearset), gear_ratio);
        let right_source =
            MotorGroupVelocity::new(MotorVelocityTracker::new(right.clone(), gearset), gear_ratio);

        Self {
            left,
            right,
            left_source,
            right_source,
            config,
            prev_time: None,
            prev_left_target: 0.0,
            prev_right_target: 0.0,
        }
    }
}

impl<FF, FB, S> VelocityDifferential<FF, FB, S> {
    /// Uses custom per-side [`WheelVelocity`] feedback sources.
    // Deliberate public plug-point; not exercised by the default wiring.
    #[allow(dead_code)]
    pub fn with_sources(
        left: Rc<RefCell<dyn AsMut<[Motor]>>>,
        right: Rc<RefCell<dyn AsMut<[Motor]>>>,
        left_source: S,
        right_source: S,
        config: VelocityDifferentialConfig<FF, FB>,
    ) -> Self {
        Self {
            left,
            right,
            left_source,
            right_source,
            config,
            prev_time: None,
            prev_left_target: 0.0,
            prev_right_target: 0.0,
        }
    }

    /// Wheel radius, in inches.
    fn wheel_radius(&self) -> f64 {
        self.config.wheel_diameter / 2.0
    }
}

/// Splits a robot-frame command into per-side linear wheel velocities.
///
/// `steer` is counterclockwise-positive, so the right side takes the positive
/// half and the left the negative one. Pulled out of `drive_arcade` so the sign
/// convention is covered by a test rather than resting on a comment.
fn arcade_to_sides(throttle: f64, steer: f64, half_track: f64) -> (f64, f64) {
    (throttle - steer * half_track, throttle + steer * half_track)
}

/// Clamps `volts` to each motor's range and applies it, returning the last error.
fn apply_side_voltage(motors: &mut [Motor], volts: f64) -> Result<(), PortError> {
    let mut result = Ok(());
    for motor in motors.iter_mut() {
        let limit = motor.max_voltage();
        if let Err(err) = motor.set_voltage(volts.clamp(-limit, limit)) {
            result = Err(err);
        }
    }
    result
}

impl<FF, FB, S> DrivetrainModel for VelocityDifferential<FF, FB, S>
where
    FF: Feedforward<State = MotorFeedforwardSetpoint, Signal = f64>,
    FB: Feedback<State = f64, Signal = f64>,
    S: WheelVelocity,
{
    type Error = PortError;
}

impl<FF, FB, S> VelocityDifferential<FF, FB, S>
where
    FF: Feedforward<State = MotorFeedforwardSetpoint, Signal = f64>,
    FB: Feedback<State = f64, Signal = f64>,
    S: WheelVelocity,
{
    /// The shared core of [`drive_arcade`](Arcade::drive_arcade) and
    /// [`drive_tank_velocity`](TankVelocity::drive_tank_velocity): converts
    /// per-side *linear* wheel velocities (in/s) into wheel *angular* velocities
    /// (rad/s) and runs each side's feedforward and feedback into a voltage.
    ///
    /// `acceleration` is the matching per-side linear acceleration (in/s^2) when
    /// the caller knows it. Passing `None` finite-differences the angular
    /// velocity target against the previous call instead, which is all
    /// `drive_arcade` can do.
    fn drive_sides(
        &mut self,
        left_linear: f64,
        right_linear: f64,
        acceleration: Option<(f64, f64)>,
    ) -> Result<(), PortError> {
        let dt = self
            .prev_time
            .map(|prev| prev.elapsed())
            .unwrap_or(Duration::from_millis(5));
        let dt_secs = dt.as_secs_f64();

        let radius = self.wheel_radius();
        // Guard against an un-configured (zero) wheel diameter.
        let (mut left_target, mut right_target) = if radius > 0.0 {
            (left_linear / radius, right_linear / radius)
        } else {
            (0.0, 0.0)
        };

        // Keep turns from demanding more than the wheels can deliver.
        if self.config.max_velocity > 0.0 {
            [left_target, right_target] =
                desaturate([left_target, right_target], self.config.max_velocity);
        }

        // Acceleration setpoint for the feedforward `ka` term. A caller-supplied
        // value wins: a motion profile's acceleration is exact, where the finite
        // difference is a one-tick-late estimate that also picks up the velocity
        // target's noise. Desaturation deliberately does not rescale it. Once the
        // wheels are saturated the profile is already being violated, and a
        // slightly optimistic `ka` term is the smaller of the two problems.
        let (left_accel, right_accel) = match acceleration {
            Some((left, right)) if radius > 0.0 => (left / radius, right / radius),
            Some(_) => (0.0, 0.0),
            // Skipped on the first tick, which has no previous target.
            None if self.prev_time.is_some() && dt_secs > 0.0 => (
                (left_target - self.prev_left_target) / dt_secs,
                (right_target - self.prev_right_target) / dt_secs,
            ),
            None => (0.0, 0.0),
        };

        // Current per-side velocity feedback, a cheap read of the trackers'
        // shared cells (does not touch the motors).
        let left_measured = self.left_source.velocity();
        let right_measured = self.right_source.velocity();

        let left_volts = side_voltage(
            self.config.left_velocity_feedforward.as_mut(),
            self.config.left_velocity_feedback.as_mut(),
            left_measured,
            left_target,
            left_accel,
            dt,
        );
        let right_volts = side_voltage(
            self.config.right_velocity_feedforward.as_mut(),
            self.config.right_velocity_feedback.as_mut(),
            right_measured,
            right_target,
            right_accel,
            dt,
        );

        let mut result = Ok(());
        {
            let mut left = self.left.borrow_mut();
            if let Err(err) = apply_side_voltage(left.as_mut(), left_volts) {
                result = Err(err);
            }
        }
        {
            let mut right = self.right.borrow_mut();
            if let Err(err) = apply_side_voltage(right.as_mut(), right_volts) {
                result = Err(err);
            }
        }

        self.prev_time = Some(Instant::now());
        self.prev_left_target = left_target;
        self.prev_right_target = right_target;

        result
    }
}

// We implement `Arcade` directly rather than `Tank`: evian's blanket
// `impl<T: Tank> Arcade for T` desaturates to 1.0 (normalized power), which would
// destroy our absolute feedforward voltages. This means `pursuit`, which requires
// `Tank`, is unavailable with this model.
impl<FF, FB, S> Arcade for VelocityDifferential<FF, FB, S>
where
    FF: Feedforward<State = MotorFeedforwardSetpoint, Signal = f64>,
    FB: Feedback<State = f64, Signal = f64>,
    S: WheelVelocity,
{
    /// - `throttle`: desired linear velocity, in inches / second.
    /// - `steer`: desired robot angular velocity, in radians / second,
    ///   counterclockwise positive. See the module docs for why, and for which
    ///   evian motions agree.
    fn drive_arcade(&mut self, throttle: f64, steer: f64) -> Result<(), Self::Error> {
        let (left, right) = arcade_to_sides(throttle, steer, self.config.track_width / 2.0);
        self.drive_sides(left, right, None)
    }
}

impl<FF, FB, S> TankVelocity for VelocityDifferential<FF, FB, S>
where
    FF: Feedforward<State = MotorFeedforwardSetpoint, Signal = f64>,
    FB: Feedback<State = f64, Signal = f64>,
    S: WheelVelocity,
{
    fn drive_tank_velocity(
        &mut self,
        left: WheelSetpoint,
        right: WheelSetpoint,
    ) -> Result<(), Self::Error> {
        self.drive_sides(
            left.velocity,
            right.velocity,
            Some((left.acceleration, right.acceleration)),
        )
    }
}

/// Sums the (optional) feedforward and (optional) feedback contributions for one
/// side into a voltage. A missing half contributes `0.0`.
fn side_voltage<FF, FB>(
    feedforward: Option<&mut FF>,
    feedback: Option<&mut FB>,
    measured: f64,
    target_velocity: f64,
    target_acceleration: f64,
    dt: Duration,
) -> f64
where
    FF: Feedforward<State = MotorFeedforwardSetpoint, Signal = f64>,
    FB: Feedback<State = f64, Signal = f64>,
{
    let ff = match feedforward {
        // `MotorFeedforward` computes `ks * velocity.signum()`, and Rust's
        // `f64::signum` returns `1.0` for `0.0`. Left alone, commanding a full
        // stop would hold `ks` volts on the motors and creep the robot forward.
        // A side asked for neither velocity nor acceleration gets no push.
        Some(_) if target_velocity == 0.0 && target_acceleration == 0.0 => 0.0,
        Some(controller) => controller.update(
            MotorFeedforwardSetpoint {
                velocity: target_velocity,
                acceleration: target_acceleration,
            },
            dt,
        ),
        None => 0.0,
    };
    let fb = match feedback {
        Some(controller) => controller.update(measured, target_velocity, dt),
        None => 0.0,
    };
    ff + fb
}

#[cfg(test)]
mod tests {
    use super::arcade_to_sides;

    #[test]
    fn straight_drives_both_sides_equally() {
        assert_eq!(arcade_to_sides(12.0, 0.0, 6.0), (12.0, 12.0));
    }

    #[test]
    fn positive_steer_turns_counterclockwise() {
        // Turning left: the right wheels outrun the left.
        let (left, right) = arcade_to_sides(10.0, 1.0, 6.0);
        assert!(right > left);
        assert_eq!((left, right), (4.0, 16.0));
    }

    #[test]
    fn steer_alone_spins_in_place() {
        let (left, right) = arcade_to_sides(0.0, 2.0, 6.0);
        assert_eq!((left, right), (-12.0, 12.0));
    }
}
