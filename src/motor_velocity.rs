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

use std::{cell::RefCell, rc::Rc};

use vexide::{
    math::Direction,
    prelude::sleep,
    smart::{motor::Motor, SmartDevice},
    task::Task,
};

use crate::{
    sensor::{TimestampedPosition, MOTOR_RAW_POSITION_RESPECTS_DIRECTION},
    velocity_estimator::VelocityEstimator,
};

/// Fallback output-shaft free speed (blue cartridge) used if a motor's gearset
/// can't be read while building its estimator.
const DEFAULT_GEARSET_RPM: f64 = 600.0;

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
    pub fn new(motors: Rc<RefCell<dyn AsMut<[Motor]>>>) -> Self {
        // Build one estimator per motor, seeding each with its own gearset speed.
        let mut estimators = Vec::new();
        {
            let mut borrow = motors.borrow_mut();
            for motor in borrow.as_mut().iter() {
                let gearset_rpm = motor
                    .gearset()
                    .map(|gearset| gearset.max_rpm())
                    .unwrap_or(DEFAULT_GEARSET_RPM);
                estimators.push(VelocityEstimator::new(gearset_rpm));
            }
        }

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
