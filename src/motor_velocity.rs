//! Background velocity tracking for a group of drive motors.
//!
//! [`MotorVelocityTracker`] owns one
//! [`VelocityEstimator`](crate::velocity_estimator::VelocityEstimator) per motor
//! and runs them from a background task, publishing the latest per-motor
//! output-shaft RPM into a shared cell that the drivetrain's velocity feedback
//! reads without touching the motors directly.
//!
//! Sharing the motors' `RefCell` with the drivetrain means the borrow to sample
//! positions must never be held across an `.await`, or it would collide with the
//! drivetrain's borrow to write voltages.
//!
//! # Sampling vs. consumption
//!
//! The estimator is sampled by this fixed-rate background task, so a controller
//! reading the shared cell may see a value up to one poll period old. This is
//! deliberate. The filter windows are sample-count based, so their time constants
//! are set by the poll rate; driving them from a control loop whose period varies
//! would make every time constant drift with loop load. The staleness is bounded
//! below the motor's data interval and is not significant.

use std::{cell::RefCell, f64::consts::PI, rc::Rc};

use vexide::{
    math::Direction,
    prelude::sleep,
    smart::{
        motor::{Gearset, Motor},
        SmartDevice,
    },
    task::Task,
};

use crate::{
    sensor::{TimestampedPosition, MOTOR_RAW_POSITION_RESPECTS_DIRECTION},
    velocity_estimator::VelocityEstimator,
};

/// Running sum and count of the live (`Some`) per-motor output-shaft RPM
/// readings, skipping failed reads so a `None` never drags the mean toward a
/// stale value. Shared by the drivetrain's velocity feedback and the sysid
/// collector so both average the group identically.
pub(crate) fn live_rpm_sum(velocities: &[Option<f64>]) -> (f64, usize) {
    let mut sum = 0.0;
    let mut count = 0;
    for &rpm in velocities.iter().flatten() {
        sum += rpm;
        count += 1;
    }
    (sum, count)
}

/// Converts a mean motor output-shaft RPM to wheel angular velocity (rad/s):
/// motor output RPM -> wheel RPM (via the external `gear_ratio`) -> rad/s.
pub(crate) fn wheel_omega_from_rpm(mean_rpm: f64, gear_ratio: f64) -> f64 {
    mean_rpm * gear_ratio * (2.0 * PI / 60.0)
}

/// Runs a [`VelocityEstimator`] per motor on a background task, exposing the
/// latest per-motor output-shaft RPM through a shared cell. An entry is `None`
/// while its motor's most recent read failed (e.g. unplugged), so consumers can
/// exclude it rather than average a stale value.
pub struct MotorVelocityTracker {
    velocities: Rc<RefCell<Vec<Option<f64>>>>,
    /// The tracking task, held so it's stopped when the tracker is dropped
    /// rather than leaked via `detach()`. Never read directly.
    _task: Task<()>,
}

impl MotorVelocityTracker {
    /// Spawns the tracking task over the shared `motors`. Sample order matches
    /// the motor slice order.
    ///
    /// `gearset` is supplied by the caller rather than read from each motor, so a
    /// motor that hasn't enumerated yet at power-on can't be silently mis-scaled
    /// by a failed `gearset()` read. This assumes every motor in the group shares
    /// the same gearset.
    pub fn new(motors: Rc<RefCell<dyn AsMut<[Motor]>>>, gearset: Gearset) -> Self {
        // One estimator per motor, all seeded with the caller-supplied output
        // free speed for the shared gearset.
        let gearset_rpm = gearset.max_rpm();
        let count = motors.borrow_mut().as_mut().len();
        let estimators: Vec<_> = (0..count)
            .map(|_| VelocityEstimator::new(gearset_rpm))
            .collect();

        let velocities = Rc::new(RefCell::new(vec![None; estimators.len()]));

        let task_motors = motors.clone();
        let task_velocities = velocities.clone();
        let task = vexide::task::spawn(async move {
            let mut estimators = estimators;
            loop {
                // Poll at twice the ~10 ms publish rate: at exactly the publish
                // rate, loop overhead would periodically read one packet twice
                // and skip the next. Oversampling avoids that; the estimator's
                // `dt == 0` early return discards the redundant reads for free.
                sleep(Motor::UPDATE_INTERVAL / 2).await;

                // Borrow only for the synchronous update; drop everything before
                // the next `.await` so the drivetrain can borrow to drive.
                let mut motors = task_motors.borrow_mut();
                let mut results = task_velocities.borrow_mut();
                for (index, motor) in motors.as_mut().iter().enumerate() {
                    // A failed read publishes `None` so consumers drop this motor
                    // from the mean instead of averaging a stale value.
                    let Ok((ticks, timestamp)) = motor.timestamped_position() else {
                        results[index] = None;
                        continue;
                    };
                    let mut rpm = estimators[index].update(ticks, timestamp);
                    if !MOTOR_RAW_POSITION_RESPECTS_DIRECTION
                        && matches!(motor.direction(), Ok(Direction::Reverse))
                    {
                        rpm = -rpm;
                    }
                    results[index] = Some(rpm);
                }
            }
        });

        Self {
            velocities,
            _task: task,
        }
    }

    /// Runs `f` over the latest per-motor output-shaft RPM (`None` where the last
    /// read failed) without cloning the shared `Rc` — for hot-path readers called
    /// every control iteration.
    pub fn with_velocities<R>(&self, f: impl FnOnce(&[Option<f64>]) -> R) -> R {
        f(&self.velocities.borrow())
    }
}
