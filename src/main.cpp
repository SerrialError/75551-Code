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
    ff_constants m1_motor_constants = {0.00126972964636, 0.15638764945, 0.273219119397};
    pros::Motor m2(-13, pros::v5::MotorGears::blue);
    ff_constants m2_motor_constants = {0.0102389892548, 0.160984984415, 0.6346622914};
    pros::Motor o1(2, pros::v5::MotorGears::blue);
    ff_constants o1_motor_constants = {0.0280821146715, 0.173093964818, 1.07228658995};
    pros::Motor o2(-12, pros::v5::MotorGears::blue);
    ff_constants o2_motor_constants = {0.00241235016359, 0.161022522912, 0.636826510544};
    pros::Motor m3(1, pros::v5::MotorGears::blue);
    ff_constants m3_motor_constants = {0.00961670602185, 0.170353286945, 0.984054974167};
    pros::Motor m4(-11, pros::v5::MotorGears::blue);
    ff_constants m4_motor_constants = {0.0249758005297, 0.180763155058, 0.750449736604};
    double wheelbase = .292100005; // m
    double trackwidth = .29508135; // m
    drivetrain mecanum(m1, m2, o1, o2, m3, m4, wheelbase, trackwidth, m1_motor_constants, m2_motor_constants, o1_motor_constants, o2_motor_constants, m3_motor_constants, m4_motor_constants);

}
