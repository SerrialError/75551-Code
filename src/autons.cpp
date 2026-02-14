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
	chassis.pid_drive_set(9.531_in, DRIVE_SPEED);
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
	chassis.pid_drive_set(15.767_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(13.631_in, DRIVE_SPEED);
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
	chassis.pid_drive_set(7.831_in, DRIVE_SPEED);
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
	chassis.pid_drive_set(16.167_in, DRIVE_SPEED);
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

void left_both_sides() {
	chassis.drive_angle_set(0_deg);
	chassis.pid_drive_set(6.527_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(22.761_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(316.329_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(32.961_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(161.255_deg, TURN_SPEED);
	chassis.pid_wait();
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(74.133_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(225.508_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(33.397_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
}

void left_both_sides_no_double() {
	chassis.drive_angle_set(0_deg);
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
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(161.255_deg, TURN_SPEED);
	chassis.pid_wait();
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(74.133_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(225.508_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(33.397_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
}

void left_single_long_goal() {
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
	chassis.pid_drive_set(30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
}

void left_double_long_goal() {
	chassis.drive_angle_set(0_deg);
	chassis.pid_drive_set(6.527_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(22.761_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(316.329_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(32.961_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.531_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
}

void right_double_long_goal() {
	chassis.drive_angle_set(180_deg);
	chassis.pid_drive_set(6.527_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(22.761_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(223.671_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(32.961_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.731_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_drive_set(30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
}

void right_single_long_goal() {
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
	chassis.pid_drive_set(30.839_in, DRIVE_SPEED);
	chassis.pid_wait();
}

void move_slight() {
	chassis.pid_drive_set(9.311_in, DRIVE_SPEED);
	chassis.pid_wait();
}
