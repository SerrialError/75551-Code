use std::{cell::RefCell, rc::Rc, time::Duration};

use evian::prelude::*;
use vexide::prelude::*;

use evian::{
    control::loops::{AngularPid, MotorFeedforward, Pid},
    drivetrain::model::Arcade,
    motion::{Basic, Seeking},
    tracking::wheeled::{TrackingWheel, WheeledTracking},
};

mod filters;
mod motor_velocity;
mod sensor;
mod velocity_estimator;

mod motion_profile;
use motion_profile::ProfileConfig;

mod profiles;

mod velocity_differential;
use velocity_differential::{MotorGroupVelocity, VelocityDifferential, VelocityDifferentialConfig};

mod sysid;
use sysid::SysIdConfig;

/// Set to `true` to run the feedforward system-identification collector
/// (`sysid::collect`) instead of the normal competition code. It drives a
/// forward-then-reverse voltage staircase and, when done, prints the data as
/// Desmos list literals; copy each block into Desmos to fit `Ks`, `Kv`, and
/// `Ka` (see `sysid.rs`). Flip back to `false` afterwards.
const RUN_SYSID: bool = false;

/// Set to `true` to run the one-shot hardware diagnostic
/// (`sensor::probe_direction`) against the first left-side motor instead of the
/// normal competition code. It reports whether `raw_position()` honors the
/// direction flag and the measured ticks per internal revolution, then exits.
/// Flip back to `false` afterwards.
const RUN_PROBE: bool = false;

struct Robot {
    drivetrain:
        Drivetrain<VelocityDifferential<MotorFeedforward, Pid, MotorGroupVelocity>, WheeledTracking>,
    controller: Controller,
}

impl Robot {
    // TODO: tune the outer linear position PID (kp, ki, kd) for this robot.
    const LINEAR_PID: Pid = Pid::new(0.0, 0.0, 0.0, None);
    // TODO: tune the outer angular (heading) PID (kp, ki, kd) for this robot.
    const ANGULAR_PID: AngularPid = AngularPid::new(0.0, 0.0, 0.0, None);
    // TODO: set the linear settling tolerances — error (inches), velocity
    // (in/s), and settle duration.
    const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(0.0)
        .velocity(0.0)
        .duration(Duration::from_millis(0));
    // TODO: set the angular settling tolerances — error (radians), velocity
    // (rad/s), and settle duration.
    const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(f64::to_radians(0.0))
        .velocity(0.0)
        .duration(Duration::from_millis(0));

    /// Full-stick linear velocity for teleop, in inches / second.
    // TODO: set the teleop full-stick linear velocity (in/s).
    const MAX_LINEAR_VELOCITY: f64 = 0.0;
    /// Full-stick angular velocity for teleop, in radians / second.
    // TODO: set the teleop full-stick angular velocity (rad/s).
    const MAX_ANGULAR_VELOCITY: f64 = 0.0;

    /// Corrections applied on top of a motion profile's feedforward.
    ///
    /// Tune these *after* the feedforward and the inner velocity loop, and with
    /// a profile the robot already roughly tracks open loop. Both start at zero,
    /// which replays the profile with no correction at all. That is a useful first
    /// run: how far off it lands tells you whether the feedforward is the thing
    /// that actually needs work.
    // TODO: tune the profile forward-travel correction (in/s per inch of error).
    const PROFILE_LINEAR_PID: Pid = Pid::new(0.0, 0.0, 0.0, None);
    // TODO: tune the profile heading correction (rad/s per radian of error).
    const PROFILE_ANGULAR_PID: AngularPid = AngularPid::new(0.0, 0.0, 0.0, None);
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let dt = &mut self.drivetrain;
        let mut seeking = Seeking {
            linear_controller: Pid::new(0.0, 0.0, 0.0, None),
            lateral_controller: Pid::new(0.0, 0.0, 0.0, None),
            tolerances: Self::LINEAR_TOLERANCES,
            timeout: Some(Duration::from_secs(10)),
        };
        let mut basic = Basic {
            linear_controller: Self::LINEAR_PID,
            angular_controller: Self::ANGULAR_PID,
            linear_tolerances: Self::LINEAR_TOLERANCES,
            angular_tolerances: Self::ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(10)),
        };

        // TODO: this is a placeholder demonstration path — replace it with the
        // real autonomous routine. Every distance (inches), heading, point, and
        // per-call override below is zeroed and needs to be set.
        basic
            .drive_distance(dt, 0.0)
            .with_linear_output_limit(0.0)
            .await;

        basic.turn_to_heading(dt, 0.0.deg()).await;

        seeking.move_to_point(dt, (0.0, 0.0)).await;

        basic
            .drive_distance_at_heading(dt, 0.0, 0.0.deg())
            .with_linear_kd(0.0)
            .with_angular_tolerance_duration(Duration::from_millis(0))
            .with_angular_error_tolerance(f64::to_radians(0.0))
            .with_linear_error_tolerance(0.0)
            .await;

        // Replay a vmplib motion profile. `profiles::example` is placeholder
        // data; point this at the module generated for the real path. The
        // profile starts from wherever the robot is now, so whatever ran before
        // it has to have settled.
        let mut profile = ProfileConfig {
            linear_controller: Some(Self::PROFILE_LINEAR_PID),
            angular_controller: Some(Self::PROFILE_ANGULAR_PID),
            wheel_units_per_meter: motion_profile::INCHES_PER_METER,
            update_interval: Motor::WRITE_INTERVAL,
        };
        _ = motion_profile::follow(dt, profiles::example::SAMPLES, &mut profile).await;
    }

    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();

            // Sticks scale to a target velocity, driven through the same
            // cascade as autonomous.
            //
            // The x-axis is negated because `drive_arcade`'s `steer` is
            // counterclockwise-positive while a stick pushed right (positive x)
            // is the driver asking to turn right, which is clockwise.
            let linear_velocity = state.left_stick.y() * Self::MAX_LINEAR_VELOCITY;
            let angular_velocity = -state.left_stick.x() * Self::MAX_ANGULAR_VELOCITY;

            _ = self
                .drivetrain
                .model
                .drive_arcade(linear_velocity, angular_velocity);
            println!("{}", self.drivetrain.tracking.position());

            sleep(Motor::WRITE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let forwards_enc = AdiOpticalEncoder::new(peripherals.adi_a, peripherals.adi_b);
    let sideways_enc = AdiOpticalEncoder::new(peripherals.adi_c, peripherals.adi_d);
    let mut left_motors = [
        Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
    ];
    let right_motors = [
        Motor::new(peripherals.port_17, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
    ];

    // Hardware diagnostic: probe one motor for the direction/ticks constants,
    // then exit. Runs before the motors are shared, so it never contends for a
    // borrow. No drivetrain model or IMU needed.
    if RUN_PROBE {
        let _ = sensor::probe_direction(&mut left_motors[0]).await;
        return;
    }

    // Shared ownership of each side's motors: the sysid collector and the
    // drivetrain's background velocity trackers both drive these through the
    // same `Rc<RefCell<..>>`.
    let left: Rc<RefCell<dyn AsMut<[Motor]>>> = Rc::new(RefCell::new(left_motors));
    let right: Rc<RefCell<dyn AsMut<[Motor]>>> = Rc::new(RefCell::new(right_motors));

    // System-identification collector: raw-voltage staircase, no drivetrain
    // model or IMU needed. Runs to completion, prints Desmos lists, then exits.
    if RUN_SYSID {
        sysid::collect(
            left.clone(),
            right.clone(),
            Gearset::Blue,
            &SysIdConfig {
                // TODO: set this to the drivetrain's real wheel-per-motor gear
                // ratio (the same value passed to `VelocityDifferential::new`
                // below) so the fitted constants are in the controller's units.
                gear_ratio: 1.0,
                ..SysIdConfig::default()
            },
        )
        .await;
        return;
    }

    let mut imu = InertialSensor::new(peripherals.port_15);
    imu.calibrate().await.unwrap();

    Robot {
        drivetrain: Drivetrain::new(
            VelocityDifferential::new(
                left.clone(),
                right.clone(),
                // gear_ratio: wheel revs per motor output-shaft rev; 1.0 for
                // direct drive.
                0.0,
                Gearset::Blue,
                // TODO: characterize the drivetrain and fill these in. Tune the
                // feedforward first, then the velocity feedback, then the outer
                // position PIDs above. Gains are in radians / second.
                VelocityDifferentialConfig {
                    left_velocity_feedforward: Some(MotorFeedforward::new(0.0, 0.0, 0.0)),
                    right_velocity_feedforward: Some(MotorFeedforward::new(0.0, 0.0, 0.0)),
                    left_velocity_feedback: Some(Pid::new(0.0, 0.0, 0.0, None)),
                    right_velocity_feedback: Some(Pid::new(0.0, 0.0, 0.0, None)),
                    wheel_diameter: 0.0,
                    track_width: 0.0,
                    max_velocity: 0.0,
                },
            ),
            // TODO: set the starting pose (position in inches, heading) and the
            // tracking-wheel geometry (wheel diameter and offset, in inches).
            WheeledTracking::new(
                (0.0, 0.0),
                0.0.deg(),
                [TrackingWheel::new(forwards_enc, 0.0, 0.0, None)],
                [TrackingWheel::new(sideways_enc, 0.0, 0.0, None)],
                Some(imu),
            ),
        ),
        controller: peripherals.primary_controller,
    }
    .compete()
    .await;
}
