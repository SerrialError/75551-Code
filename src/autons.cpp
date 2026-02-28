#include "main.h"

void skills_left_one() {
	chassis.drive_angle_set(0_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.131_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(1500);
}

void skills_park_only() {
	chassis.drive_angle_set(0_deg);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(21.614_in, DRIVE_SPEED);
	chassis.pid_wait();
}

void skills_all_match_loader_park() {
	chassis.drive_angle_set(0_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.331_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(134.287_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(69.317_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(45.713_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.667_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(14.531_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(225.713_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(180_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(70.75_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(134.287_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.167_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(8.831_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(314.287_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(69.317_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(225.713_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(17.267_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(14.131_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(12.2_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(0_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(46.257_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(29.885_in, 80);
	chassis.pid_wait();
}

void left_long_rush() {
	chassis.drive_angle_set(0_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(9.339_in, DRIVE_SPEED);
	Intake.intakeState = intakeOff;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_turn_set(154.284_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(11.772_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	wings.extend();
	chassis.pid_wait();
	chassis.pid_drive_set(16.861_in, DRIVE_SPEED);
	chassis.pid_wait();
}

void left_long_middle() {
	chassis.drive_angle_set(0_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(134.492_deg, TURN_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(33.397_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(315.625_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-11.085_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = midScore;
	Intake.update_intake_state();
}

void left_awp() {
	chassis.drive_angle_set(0_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(134.492_deg, TURN_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(33.397_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(315.625_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-11.085_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = midScore;
	Intake.update_intake_state();
	pros::delay(1000);
	chassis.pid_drive_set(11.085_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(198.745_deg, TURN_SPEED);
	Intake.intakeState = intakeOff;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(74.133_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
}

void right_small_long_middle() {
	chassis.drive_angle_set(180_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(18.745_deg, TURN_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(74.133_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(315.625_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-11.085_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = midScore;
	Intake.update_intake_state();
}

void right_long_middle() {
	chassis.drive_angle_set(180_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(45.508_deg, TURN_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(33.397_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(0_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(46.796_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(315.625_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-11.085_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = midScore;
	Intake.update_intake_state();
}

void right_small_awp() {
	chassis.drive_angle_set(180_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(18.745_deg, TURN_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(74.133_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(315.625_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-11.085_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = midScore;
	Intake.update_intake_state();
	chassis.pid_drive_set(44.48_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
}

void right_large_awp() {
	chassis.drive_angle_set(180_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(45.508_deg, TURN_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(33.397_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(0_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(46.796_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(315.625_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-11.085_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = midScore;
	Intake.update_intake_state();
	chassis.pid_drive_set(44.48_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
}

void right_long_rush() {
	chassis.drive_angle_set(180_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(9.339_in, DRIVE_SPEED);
	Intake.intakeState = intakeOff;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_turn_set(25.716_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(11.772_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	wings.extend();
	chassis.pid_wait();
	chassis.pid_drive_set(16.861_in, DRIVE_SPEED);
	chassis.pid_wait();
}

void move_slight() {
	chassis.pid_drive_set(9.311_in, DRIVE_SPEED);
	chassis.pid_wait();
}