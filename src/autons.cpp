#include "main.h"

void skills_all_match_loader_park_control_zone() {
	chassis.drive_angle_set(0_deg);
	chassis.pid_drive_set(30.368_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	match_loader.extend(); // down
	chassis.pid_wait();
	chassis.pid_drive_set(9.031_in, SKILLS_DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-9.331_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	match_loader.retract(); // up
	chassis.pid_turn_set(134.287_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(69.317_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(45.713_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(15.567_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-20.708_in, 127);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
	match_loader.extend(); // down
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(30.339_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.339_in, 127);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
	match_loader.retract(); // up
	chassis.pid_drive_set(20.708_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(225.713_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(180_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(70.75_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(134.287_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, SKILLS_TURN_SPEED);
	match_loader.extend(); // down
	chassis.pid_wait();
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(6.631_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	pros::delay(2000);
	match_loader.retract(); // up
	chassis.pid_drive_set(-8.631_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(314.287_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(69.317_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(225.713_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-20.708_in, 127);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
	match_loader.extend(); // down
	chassis.pid_wait();
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(30.339_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.339_in, 127);
	chassis.pid_wait();
	match_loader.retract(); // up
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
	chassis.pid_drive_set(12.2_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(313.065_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(28.922_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(351.29_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(27.025_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
}

void skills_and_mid() {
	chassis.drive_angle_set(0_deg);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(39.175_in, PARK_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(49.939_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(5.651_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90.21_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(36.218_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(315.624_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-19.882_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
    Intake.backStageSpeed = 0.5;
	Intake.intakeState = midScore;
	Intake.update_intake_state();
	pros::delay(1000);
	Intake.update_intake_state();
	Intake.backStageSpeed = 1.0;
	chassis.pid_drive_set(53.276_in, SKILLS_DRIVE_SPEED);
	Intake.intakeState = intakeOff;
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, SKILLS_TURN_SPEED);
	match_loader.extend(); // down
	chassis.pid_wait();
	chassis.pid_drive_set(9.031_in, SKILLS_DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-9.331_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	match_loader.retract(); // up
	chassis.pid_turn_set(134.287_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(69.317_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(45.713_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(15.567_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-20.708_in, 127);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
	match_loader.extend(); // down
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(30.339_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.339_in, 127);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
	match_loader.retract(); // up
	chassis.pid_drive_set(20.708_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(225.713_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(180_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(70.75_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(134.287_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(90_deg, SKILLS_TURN_SPEED);
	match_loader.extend(); // down
	chassis.pid_wait();
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(6.631_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	pros::delay(2000);
	match_loader.retract(); // up
	chassis.pid_drive_set(-8.631_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(314.287_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(69.317_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(225.713_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(16.367_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-20.708_in, 127);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
	match_loader.extend(); // down
	chassis.pid_wait();
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_drive_set(30.339_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.339_in, 127);
	chassis.pid_wait();
	match_loader.retract(); // up
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
	chassis.pid_drive_set(12.2_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(313.065_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(28.922_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(351.29_deg, SKILLS_TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(27.025_in, SKILLS_DRIVE_SPEED);
	chassis.pid_wait();
}

void left_long_rush() {
	chassis.drive_angle_set(0_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	match_loader.extend(); // down
	chassis.pid_wait();
	chassis.pid_drive_set(9.031_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(-30.339_in, 127);
	chassis.pid_wait();
	match_loader.retract(); // up
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
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
	match_loader.extend(); // down
	chassis.pid_wait();
	chassis.pid_drive_set(9.031_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(-30.339_in, 127);
	chassis.pid_wait();
	match_loader.retract(); // up
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
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
	chassis.pid_drive_set(-19.782_in, 127);
	chassis.pid_wait();
	Intake.intakeState = midScore;
	Intake.update_intake_state();
}

void left_awp() {
	chassis.drive_angle_set(0_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	match_loader.extend(); // down
	chassis.pid_wait();
	chassis.pid_drive_set(9.031_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(-30.339_in, 127);
	chassis.pid_wait();
	match_loader.retract(); // up
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
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
	chassis.pid_drive_set(-19.782_in, 127);
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
	chassis.pid_drive_set(9.031_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.339_in, 127);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
}

void left_large_awp() {
	chassis.drive_angle_set(0_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	match_loader.extend(); // down
	chassis.pid_wait();
	chassis.pid_drive_set(9.031_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(-30.339_in, 127);
	chassis.pid_wait();
	match_loader.retract(); // up
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(1000);
	chassis.pid_drive_set(20.708_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(134.492_deg, TURN_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_drive_set(33.397_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(315.625_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-19.782_in, 127);
	chassis.pid_wait();
	Intake.backStageSpeed = 0.5;
	Intake.intakeState = midScore;
	Intake.update_intake_state();
	pros::delay(1000);
	chassis.pid_drive_set(19.782_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	chassis.pid_turn_set(180_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(46.796_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(225.508_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(32.397_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	chassis.pid_drive_set(-21.208_in, DRIVE_SPEED);
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
}


void right_small_long_middle() {
	chassis.drive_angle_set(180_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	match_loader.extend(); // down
	chassis.pid_wait();
	pros::delay(200);
	chassis.pid_drive_set(9.031_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.339_in, 127);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
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
	chassis.pid_drive_set(-19.782_in, 127);
	chassis.pid_wait();
	Intake.intakeState = midScore;
	Intake.update_intake_state();
}

void right_long_middle() {
	chassis.drive_angle_set(180_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	match_loader.extend(); // down
	chassis.pid_wait();
	pros::delay(200);
	chassis.pid_drive_set(9.031_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.339_in, 127);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
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
	chassis.pid_drive_set(-19.782_in, 127);
	chassis.pid_wait();
	Intake.intakeState = midScore;
	Intake.update_intake_state();
}

void right_small_awp() {
	chassis.drive_angle_set(180_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	match_loader.extend(); // down
	chassis.pid_wait();
	pros::delay(200);
	chassis.pid_drive_set(9.031_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.339_in, 127);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
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
	chassis.pid_drive_set(-19.782_in, 127);
	chassis.pid_wait();
	Intake.intakeState = midScore;
	Intake.update_intake_state();
	chassis.pid_drive_set(44.48_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.031_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.339_in, 127);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
}

void right_large_awp() {
	chassis.drive_angle_set(180_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	match_loader.extend(); // down
	chassis.pid_wait();
	pros::delay(200);
	chassis.pid_drive_set(9.031_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.339_in, 127);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
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
	chassis.pid_drive_set(-19.782_in, 127);
	chassis.pid_wait();
	Intake.intakeState = midScore;
	Intake.update_intake_state();
	chassis.pid_drive_set(44.48_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	chassis.pid_wait();
	match_loader.extend(); // down
	pros::delay(200);
	chassis.pid_drive_set(9.031_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.339_in, 127);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
}

void right_long_rush() {
	chassis.drive_angle_set(180_deg);
	chassis.pid_drive_set(30.368_in, DRIVE_SPEED);
	chassis.pid_wait();
	chassis.pid_turn_set(270_deg, TURN_SPEED);
	match_loader.extend(); // down
	chassis.pid_wait();
	pros::delay(200);
	chassis.pid_drive_set(9.031_in, DRIVE_SPEED);
	Intake.intakeState = intakeOnly;
	Intake.update_intake_state();
	chassis.pid_wait();
	pros::delay(2000);
	chassis.pid_drive_set(-30.339_in, 127);
	match_loader.retract(); // up
	chassis.pid_wait();
	Intake.intakeState = topScore;
	Intake.update_intake_state();
	pros::delay(2500);
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


