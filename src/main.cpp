/**
 * \file main.cpp
 * @brief PROS entry points and high-level robot wiring.
 *
 * Creates the global drivetrain and intake instances, wires up sensors
 * and motors, and implements the standard PROS callbacks (`initialize`,
 * `disabled`, `competition_initialize`, `autonomous`, and `opcontrol`).
 */
#include "main.h"
pros::adi::Pneumatics match_loader ('a', true, false);
pros::Rotation linear_wheel(16);
pros::Rotation horizontal_wheel(15);
pros::Imu imu_sensor_1(3);

pros::Motor m1(-2, pros::v5::MotorGears::blue);
pros::Motor m2(14, pros::v5::MotorGears::blue);
pros::Motor o1(-13, pros::v5::MotorGears::blue);
pros::Motor o2(18, pros::v5::MotorGears::blue);
pros::Motor m3(-11, pros::v5::MotorGears::blue);
pros::Motor m4(17, pros::v5::MotorGears::blue);

// Drivetrain motor grouping: mecanum (m1–m4) plus omni wheels (o1, o2).
const wheels<std::reference_wrapper<pros::Motor>> driveMotors{
	std::ref(m1),
	std::ref(m2),
	std::ref(o1),
	std::ref(o2),
	std::ref(m3),
	std::ref(m4)
};

// kA, kV, kS, max rad/s (calculated at 12 V), max voltage
const wheels<FirstOrderFeedforwardConstants> dtFFConsts{ 
	{0.0045870351481, 0.166157827462, 0.120535260966, 75.858991, 12.24}, 
	{0.00346235990182, 0.16618573907, 0.103286863565, 74.351026, 12.055}, 
	{0.0011750526572, 0.156487380984, 0.133576249675, 80.215332, 12.055}, 
	{0.00169461151421, 0.161775364104, 0.136640169512, 76.235982, 12.055}, 
	{0.0043200416094, 0.16243797244, 0.259385827274, 75.314448, 12.012}, 
	{0.00101968096594, 0.157778384448, 0.16310482292, 78.351321, 12.012} };

const wheels<PIDConstants> dtPIDConsts{ 
	{0.f, 0.f, 0.f},
	{0.f, 0.f, 0.f}, 
	{0.f, 0.f, 0.f}, 
	{0.f, 0.f, 0.f}, 
	{0.f, 0.f, 0.f}, 
	{0.f, 0.f, 0.f} };

const float wheelbase = 0.292100005; // m
const float trackwidth = 0.29508135; // m

const float dt_ = 0.01f;

drivetrain dt(driveMotors, wheelbase, trackwidth, dt_, dtFFConsts, dtPIDConsts, linear_wheel, horizontal_wheel, imu_sensor_1, {0, 0, 0});

pros::Motor front(-5, pros::v5::MotorGears::blue);
pros::Motor back(-6, pros::v5::MotorGears::blue);

const rollers<std::reference_wrapper<pros::Motor>> intakeMotors{
	std::ref(front),
	std::ref(back)
};

const rollers<FirstOrderFeedforwardConstants> intakeConsts {
	{0.00681526983289, 0.276443936704, 0.23522177916, 70.590165, 12.618},
	{0.00981526983289, 0.276443936704, 0.23522177916, 70.590165, 12.618}
};

intake Intake(intakeMotors, intakeConsts);

autons current_auton = blueRight;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_left_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
		current_auton = static_cast<autons>((current_auton - 1) % 4);
		switch (current_auton) {
			case blueRight:
				pros::lcd::set_text(1, "Blue Right");
				break;
			case blueLeft:
				pros::lcd::set_text(1, "Blue Left");
				break;
			case redLeft:
				pros::lcd::set_text(1, "Red Left");
				break;
			case redRight:
				pros::lcd::set_text(1, "Red Right");
				break;
			default:
				pros::lcd::set_text(1, "Blue Right");
				break;
		}
    }
}

void on_right_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
		current_auton = static_cast<autons>((current_auton + 1) % 4);
		switch (current_auton) {
			case blueRight:
				pros::lcd::set_text(1, "Blue Right");
				break;
			case blueLeft:
				pros::lcd::set_text(1, "Blue Left");
				break;
			case redLeft:
				pros::lcd::set_text(1, "Red Left");
				break;
			case redRight:
				pros::lcd::set_text(1, "Red Right");
				break;
			default:
				pros::lcd::set_text(1, "Blue Right");
				break;
		}
    }
	
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


void initialize() {
    pros::lcd::initialize();
	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn2_cb(on_right_button);
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}


/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */



void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	switch (current_auton) {
		case blueRight:
			dt.localization.set_pose({0.0, 0.0, 0.0});
			Intake.intakeState = intakeOnly;
			for (int i = 0; i < 20; i++) {
	 			Intake.update_intake_state(dt_);
        			pros::delay(static_cast<int>(dt_*1000.0));
			}
			dt.linear_mp(40 * 0.0254);
			pros::delay(500);
			dt.angular_mp(162.0 * M_PI / 180.0 * 0.0254);
			pros::delay(500);
			dt.linear_mp(21.0 * 0.0254);
			Intake.intakeState = bottomScore;
			for (int i = 0; i < 20; i++) {
	 			Intake.update_intake_state(dt_);
        			pros::delay(static_cast<int>(dt_*1000.0));
			}
			break;
		case blueLeft:
			break;
		case redLeft:

			break;
		case redRight:

			break;
		default:
			
			break;
	}
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
    while(true) {
	    if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
	        Intake.intakeState = topScore;
	    }
        if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
	        Intake.intakeState = midScore; 			
	    }
        if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
	        Intake.intakeState = bottomScore; 			
	    }
        if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
	        Intake.intakeState = intakeOnly;
	    }
        if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
	        Intake.intakeState = intakeOff; 			
	    }
		if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			match_loader.retract();
	    }
		if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			match_loader.extend();
	    }
		if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
			dt.calculate_and_print_motor_constants();
		}
	 	Intake.update_intake_state(dt_);
	    // dt.field_oriented_holonomic_control(dt_);
	    dt.tank_drive_control();
        // dt.test_control(dt_);
        pros::delay(static_cast<int>(dt_*1000.0));
    }
}
