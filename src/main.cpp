#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
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
    pros::lcd::set_text(1, "Hello PROS User!");


    pros::lcd::register_btn1_cb(on_center_button);
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
void competition_initialize() {}


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
void autonomous() {}


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

    // motor_angle_mp_test(m1, m1_motor_constants, 78.497928, 4700.12687931 * 0.8f, 2.f * M_PI * 10);
    pros::Motor m1(-13, pros::v5::MotorGears::blue);
    pros::Motor m2(18, pros::v5::MotorGears::blue);
    pros::Motor o1(-10, pros::v5::MotorGears::blue);
    pros::Motor o2(19, pros::v5::MotorGears::blue);
    pros::Motor m3(-11, pros::v5::MotorGears::blue);
    pros::Motor m4(20, pros::v5::MotorGears::blue);

    const wheels<std::reference_wrapper<pros::Motor>> driveMotors{
        std::ref(m1),
        std::ref(m2),
        std::ref(o1),
        std::ref(o2),
        std::ref(m3),
        std::ref(m4)
    };

    // kA, kV, kS
    const wheels<ff_constants> dtConsts{
        {0.0045870351481, 0.166157827462, 0.120535260966, 75.858991, 12.24},
        {0.00346235990182, 0.16618573907, 0.103286863565, 74.351026, 12.055},
        {0.0011750526572, 0.156487380984, 0.133576249675, 80.215332, 12.055},
        {0.00169461151421, 0.161775364104, 0.136640169512, 76.235982, 12.055},
        {0.0043200416094, 0.16243797244, 0.259385827274, 75.314448, 12.012},
        {0.00101968096594, 0.157778384448, 0.16310482292, 78.351321, 12.012}
    };

    pros::Imu imu_sensor(3);

    const double wheelbase = 0.292100005; // m
    const double trackwidth = 0.29508135; // m

    drivetrain dt(driveMotors, imu_sensor, wheelbase, trackwidth, dtConsts);
    
    pros::Motor fb(8, pros::v5::MotorGears::blue);
    pros::Motor ft(2, pros::v5::MotorGears::blue);
    pros::Motor bb(1, pros::v5::MotorGears::blue);
    pros::Motor bt(10, pros::v5::MotorGears::blue);

    const rollers<std::reference_wrapper<pros::Motor>> intakeMotors{
        std::ref(fb),
        std::ref(ft),
        std::ref(bb),
        std::ref(bt)
    };
    // compute sysid(fb);
    // std::vector<input_output> u_vs_x = sysid.fopdt_system_identification(200);
    // print_vector(u_vs_x);
    pros::Optical optical(4);
    const rollers<ff_constants> intakeConsts {
        {0.00681526983289, 0.276443936704, 0.23522177916, 27.590165, 7.618},
        {0.00681526983289, 0.276443936704, 0.23522177916, 27.590165, 7.618},   	 
        {0.00291077896309, 0.26137479893, 0.0463373039661, 31.360076, 7.585},
        {0.00744674908847, 0.265135278752, 0.17548243974, 27.646015, 7.294}
    };

    intake Intake(intakeMotors, optical, intakeConsts, red);
    Intake.optical.set_led_pwm(75);
    Intake.optical.set_integration_time(10.0);
    const double dt_ = 0.01;
    
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
	 	Intake.update_intake_state(dt_);
	    // dt.field_oriented_holonomic_control(dt_);
	    dt.tank_drive_control(dt_);
        // dt.test_control(dt_);
        pros::delay(static_cast<int>(dt_*100.0));
    }
}
