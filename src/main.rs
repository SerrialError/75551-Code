use std::time::Duration;

use evian::prelude::*;
use vexide::prelude::*;

use evian::{
    control::loops::{AngularPid, MotorFeedforward, Pid},
    drivetrain::model::Arcade,
    motion::{Basic, Seeking},
    tracking::wheeled::{TrackingWheel, WheeledTracking},
};

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

struct Robot {
    drivetrain:
        Drivetrain<VelocityDifferential<MotorFeedforward, Pid, MotorGroupVelocity>, WheeledTracking>,
    controller: Controller,
}

impl Robot {
    const LINEAR_PID: Pid = Pid::new(1.0, 0.0, 0.125, None);
    const ANGULAR_PID: AngularPid = AngularPid::new(16.0, 0.0, 1.0, None);
    const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(4.0)
        .velocity(0.25)
        .duration(Duration::from_millis(15));
    const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(f64::to_radians(8.0))
        .velocity(0.09)
        .duration(Duration::from_millis(15));

    /// Full-stick linear velocity for teleop, in inches / second.
    const MAX_LINEAR_VELOCITY: f64 = 0.0;
    /// Full-stick angular velocity for teleop, in radians / second.
    const MAX_ANGULAR_VELOCITY: f64 = 0.0;
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

        basic
            .drive_distance(dt, 24.0)
            .with_linear_output_limit(6.0)
            .await;

        basic.turn_to_heading(dt, 0.0.deg()).await;

        seeking.move_to_point(dt, (24.0, 24.0)).await;

        basic
            .drive_distance_at_heading(dt, 8.0, 45.0.deg())
            .with_linear_kd(1.2)
            .with_angular_tolerance_duration(Duration::from_millis(5))
            .with_angular_error_tolerance(f64::to_radians(10.0))
            .with_linear_error_tolerance(12.0)
            .await;
    }

    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();

            // Sticks scale to a target velocity, driven through the same
            // cascade as autonomous.
            let linear_velocity = state.left_stick.y() * Self::MAX_LINEAR_VELOCITY;
            let angular_velocity = state.left_stick.x() * Self::MAX_ANGULAR_VELOCITY;

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
        Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
    ];
    let mut right_motors = [
        Motor::new(peripherals.port_17, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
    ];

    // System-identification collector: raw-voltage staircase, no drivetrain
    // model or IMU needed. Runs to completion, prints Desmos lists, then exits.
    if RUN_SYSID {
        sysid::collect(
            &mut left_motors,
            &mut right_motors,
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
                left_motors,
                right_motors,
                // gear_ratio: wheel revs per motor output-shaft rev; 1.0 for
                // direct drive.
                0.0,
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
            WheeledTracking::new(
                (0.0, 0.0),
                90.0.deg(),
                [TrackingWheel::new(forwards_enc, 2.0, 0.0, None)],
                [TrackingWheel::new(sideways_enc, 2.0, 0.0, None)],
                Some(imu),
            ),
        ),
        controller: peripherals.primary_controller,
    }
    .compete()
    .await;
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_adds_two() {
        assert_eq!(2 + 2, 4);
    }
}
