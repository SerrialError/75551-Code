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
    pros::Motor test_motor(7);
    ff_constants test_motor_constants = {0.00380743747817, 0.510097726504, 0.17033803784};
    // oned_motion_profiler(test_motor, test_motor_constants);
    compute sysid(test_motor);
    std::vector<input_output> u_vs_x = sysid.fopdt_system_identification(200);
    print_vector(u_vs_x);
    pros::Motor m1(1, pros::v5::MotorGears::blue);
    ff_constants m1_motor_constants = {0.00380743747817, 0.510097726504, 0.17033803784};
    pros::Motor m2(2, pros::v5::MotorGears::blue);
    ff_constants m2_motor_constants = {0.00380743747817, 0.510097726504, 0.17033803784};
    pros::Motor o1(3, pros::v5::MotorGears::blue);
    ff_constants o1_motor_constants = {0.00380743747817, 0.510097726504, 0.17033803784};
    pros::Motor o2(4, pros::v5::MotorGears::blue);
    ff_constants o2_motor_constants = {0.00380743747817, 0.510097726504, 0.17033803784};
    pros::Motor m3(5, pros::v5::MotorGears::blue);
    ff_constants m3_motor_constants = {0.00380743747817, 0.510097726504, 0.17033803784};
    pros::Motor m4(6, pros::v5::MotorGears::blue);
    ff_constants m4_motor_constants = {0.00380743747817, 0.510097726504, 0.17033803784};
    drivetrain mecanum(m1, m2, o1, o2, m3, m4, .292100005, .29508135, m1_motor_constants, m2_motor_constants, o1_motor_constants, o2_motor_constants, m3_motor_constants, m4_motor_constants);

}
