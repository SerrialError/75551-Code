//! A differential drivetrain model with a cascaded velocity controller.
//!
//! Where evian's stock `Differential` interprets its inputs as raw normalized
//! voltages, [`VelocityDifferential`] treats the motion commands' outputs as
//! *velocities* and runs an inner per-side loop that turns them into voltages:
//!
//! ```text
//! left_v  = linear_v + angular_v * (track_width / 2)   [in/s]
//! right_v = linear_v - angular_v * (track_width / 2)   [in/s]
//! target_w = side_v / wheel_radius                     [rad/s]
//! volts    = feedforward(target_w, target_a) + feedback(target_w - measured_w)
//! ```
//!
//! Each side's feedforward (`FF`) and feedback (`FB`) are independently optional;
//! a missing half contributes `0.0` volts. The velocity feedback comes from a
//! per-side [`WheelVelocity`] source (`S`); the built-in [`MotorGroupVelocity`]
//! reads the drive motors' own encoders, since `WheeledTracking` only reports
//! robot-frame velocity.
//!
//! The inner loop regulates each wheel's angular velocity in **radians / second**
//! — hence a plain `Pid` over `f64` rather than an `AngularPid`, whose `±π` error
//! wrapping is correct for a heading but nonsense for a velocity. `throttle` is in
//! inches / second, `steer` in radians / second, and lengths in inches.

use std::{
    cell::RefCell,
    f64::consts::PI,
    rc::Rc,
    time::{Duration, Instant},
};

use evian::{
    control::loops::{Feedback, Feedforward, MotorFeedforwardSetpoint},
    drivetrain::model::{Arcade, DrivetrainModel},
    math::desaturate,
};
use vexide::{prelude::Motor, smart::PortError};

/// A source of a drivetrain side's measured wheel angular velocity, in
/// radians / second.
pub trait WheelVelocity {
    fn velocity(&mut self) -> f64;
}

/// A [`WheelVelocity`] source backed by a group of drive motors' internal
/// encoders, sharing ownership of the motors with the drivetrain so they can be
/// both driven and read.
pub struct MotorGroupVelocity {
    motors: Rc<RefCell<dyn AsMut<[Motor]>>>,
    /// Wheel revolutions per motor output-shaft revolution (external gearing
    /// only; [`Motor::velocity`] already reports gearset-reduced RPM). `1.0` for
    /// direct drive.
    gear_ratio: f64,
}

impl MotorGroupVelocity {
    pub fn new(motors: Rc<RefCell<dyn AsMut<[Motor]>>>, gear_ratio: f64) -> Self {
        Self { motors, gear_ratio }
    }
}

impl WheelVelocity for MotorGroupVelocity {
    fn velocity(&mut self) -> f64 {
        let mut borrow = self.motors.borrow_mut();
        let motors = borrow.as_mut();

        let mut sum_rpm = 0.0;
        let mut count = 0.0;
        for motor in motors.iter() {
            if let Ok(rpm) = motor.velocity() {
                sum_rpm += rpm;
                count += 1.0;
            }
        }
        if count == 0.0 {
            return 0.0;
        }
        // motor output RPM -> wheel RPM -> wheel rad/s
        (sum_rpm / count) * self.gear_ratio * (2.0 * PI / 60.0)
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
    /// `gear_ratio` configures the built-in [`MotorGroupVelocity`] sources; for a
    /// custom velocity source, use [`with_sources`](Self::with_sources) instead.
    pub fn new<L, R>(
        left: L,
        right: R,
        gear_ratio: f64,
        config: VelocityDifferentialConfig<FF, FB>,
    ) -> Self
    where
        L: AsMut<[Motor]> + 'static,
        R: AsMut<[Motor]> + 'static,
    {
        let left: Rc<RefCell<dyn AsMut<[Motor]>>> = Rc::new(RefCell::new(left));
        let right: Rc<RefCell<dyn AsMut<[Motor]>>> = Rc::new(RefCell::new(right));
        let left_source = MotorGroupVelocity::new(left.clone(), gear_ratio);
        let right_source = MotorGroupVelocity::new(right.clone(), gear_ratio);

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
    pub fn with_sources<L, R>(
        left: L,
        right: R,
        left_source: S,
        right_source: S,
        config: VelocityDifferentialConfig<FF, FB>,
    ) -> Self
    where
        L: AsMut<[Motor]> + 'static,
        R: AsMut<[Motor]> + 'static,
    {
        Self {
            left: Rc::new(RefCell::new(left)),
            right: Rc::new(RefCell::new(right)),
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
    /// - `steer`: desired robot angular velocity, in radians / second.
    fn drive_arcade(&mut self, throttle: f64, steer: f64) -> Result<(), Self::Error> {
        let dt = self
            .prev_time
            .map(|prev| prev.elapsed())
            .unwrap_or(Duration::from_millis(5));
        let dt_secs = dt.as_secs_f64();

        // Combine robot-frame velocities into per-side *linear* wheel velocities
        // (in/s), then convert each to a wheel *angular* velocity (rad/s).
        let half_track = self.config.track_width / 2.0;
        let left_linear = throttle + steer * half_track;
        let right_linear = throttle - steer * half_track;

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

        // Acceleration setpoint for the feedforward `ka` term (finite difference
        // of the target angular velocity). Skipped on the first tick.
        let (left_accel, right_accel) = if self.prev_time.is_some() && dt_secs > 0.0 {
            (
                (left_target - self.prev_left_target) / dt_secs,
                (right_target - self.prev_right_target) / dt_secs,
            )
        } else {
            (0.0, 0.0)
        };

        // Read the velocity feedback *before* borrowing the motors for writing:
        // the default source shares the same motor `RefCell`, so the read and
        // the write must not overlap.
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
