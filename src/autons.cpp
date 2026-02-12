#include "main.h"

void skills_left_one() {
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(8.131_in, 90);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-28.539_in, DRIVE_SPEED);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(1500);
	chassis.pid_drive_set(21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(134.287_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
}

void skills_deload_left_park() {
	chassis.pid_drive_set(29.064_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270.189_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	chassis.pid_drive_set(6.958_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	// match load
	pros::delay(1000);
	chassis.pid_turn_set(270.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-28.545_in, DRIVE_SPEED);
	pros::delay(40);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2000);
	chassis.pid_turn_set(244.556_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(34.897_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(193.896_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(11.908_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(180.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(20.035_in, DRIVE_SPEED);
	chassis.pid_wait();
}

void skills_park_only() {
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(21.614_in, DRIVE_SPEED);
	chassis.pid_wait();
}

void skills_all_match_loader_park() {
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.131_in, 90);
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
	chassis.pid_drive_set(16.167_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(13.631_in, 90);
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
	chassis.pid_drive_set(9.131_in, 90);
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
	chassis.pid_drive_set(13.631_in, 90);
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

void skills_all_park_double_clear() {
	chassis.pid_drive_set(29.064_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(6.958_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-28.545_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(21.491_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(134.163_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(18.093_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(71.073_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(46.425_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(18.162_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(6.958_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-28.545_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(21.491_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(144.543_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(18.207_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(168.193_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(10.502_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(180.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(21.869_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(180.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(21.632_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(192.185_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(10.502_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(215.589_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(18.152_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(6.958_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-28.545_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(21.491_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(314.163_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(18.093_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(71.073_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(226.425_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(18.162_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(6.958_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-28.545_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(21.491_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(324.543_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(18.207_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(348.193_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(10.502_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(0.189_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(21.869_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(0.189_deg, TURN_SPEED);
	chassis.pid_wait();
}


void redLeft_1_side_long_goal() {
	chassis.pid_drive_set(29.064_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(6.958_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-28.545_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
}

void redRight_1_side_long_goal() {
	chassis.pid_drive_set(29.064_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(6.958_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-28.545_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
}

void blueLeft_1_side_long_goal() {
	chassis.pid_drive_set(29.064_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(6.958_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-28.545_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
}

void blueRight_1_side_long_goal() {
	chassis.pid_drive_set(29.064_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(6.958_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-28.545_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
}

void redRight_sawp() {
	chassis.pid_drive_set(29.064_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(6.958_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-28.545_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(21.587_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(47.305_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(35.513_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(0_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(46.136_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(316.236_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-13.87_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(313.447_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(49.034_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(7.054_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-27.416_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(269.706_deg, TURN_SPEED);
	chassis.pid_wait();
}

void blueRight_sawp() {
	chassis.pid_drive_set(29.064_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(6.958_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(28.545_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(21.587_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(227.305_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(35.513_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(180_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(46.136_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(316.236_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(13.87_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(133.447_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(49.034_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(7.054_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(27.416_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(89.706_deg, TURN_SPEED);
	chassis.pid_wait();
}

void move_slight() {
	chassis.pid_drive_set(9.311_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(0_deg, TURN_SPEED);
	chassis.pid_wait();
}


