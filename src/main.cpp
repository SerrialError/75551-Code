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
    // compute sysid(m4);
    // std::vector<input_output> u_vs_x = sysid.fopdt_system_identification(200);
    // print_vector(u_vs_x);
    // motor_angle_mp_test(m1, m1_motor_constants, 78.497928, 4700.12687931 * 0.8f, 2.f * M_PI * 10);
    pros::Motor m1(3, pros::v5::MotorGears::blue);
    pros::Motor m2(-13, pros::v5::MotorGears::blue);
    pros::Motor o1(2, pros::v5::MotorGears::blue);
    pros::Motor o2(-12, pros::v5::MotorGears::blue);
    pros::Motor m3(1, pros::v5::MotorGears::blue);
    pros::Motor m4(-11, pros::v5::MotorGears::blue);

    const wheels<std::reference_wrapper<pros::Motor>> motors{
        std::ref(m1),
        std::ref(m2),
        std::ref(o1),
        std::ref(o2),
        std::ref(m3),
        std::ref(m4)
    };

    // kA, kV, kS
    const wheels<ff_constants> consts{
        {0.00126972964636, 0.15638764945, 0.273219119397, 4700.12687931 * 0.9, 78.497928},  // m1
        {0.0102389892548, 0.160984984415, 0.6346622914, 72.130967 * 0.9, 11.963},   	    // m2
        {0.0280821146715, 0.173093964818, 1.07228658995, 188.531268637 * 0.9, 63.020349},   // o1
        {0.00241235016359, 0.161022522912, 0.636826510544, 2339.87040104 * 0.9, 72.947781}, // o2
        {0.00961670602185, 0.170353286945, 0.984054974167, 538.435146208 * 0.9, 63.460172}, // m3
        {0.0249758005297, 0.180763155058, 0.750449736604, 224.087917624 * 0.9, 64.674921}   // m4
    };

    pros::Imu imu_sensor(4);

    const double wheelbase = 0.292100005; // m
    const double trackwidth = 0.29508135; // m

    drivetrain dt(motors, imu_sensor, wheelbase, trackwidth, consts);
    
    while(true) {
	const double dt_ = 0.01;
	dt.field_oriented_holonomic_control(dt_);
	pros::delay(10);
    }
}
