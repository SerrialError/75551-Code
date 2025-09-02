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
pros::Motor test_motor(1);

struct input_output {
	double u;
	double x;
};

std::vector<input_output> fopdt_system_identification(int n) {
	std::vector<input_output> result;
	for (int i = -12; i < 13; i++) {
		test_motor.move_voltage(1000*i);
		for (int j = 0; j < n; j++) {            
			double measured_mv = test_motor.get_voltage();           // mV
            		double measured_rpm = test_motor.get_actual_velocity(); // RPM
            		input_output sample;
            		sample.u = measured_mv / 1000.f;             // V
            		sample.x = measured_rpm * 2.f * M_PI / 60.f; // rad/s
            		result.push_back(sample);
			pros::delay(10);
		};
	};
	test_motor.move_voltage(0);
	return result;	
};

void print_vector(const std::vector<input_output>& vec) {
    printf("U = [");
    for (size_t i = 0; i < vec.size(); ++i) {
        printf("%.6f", vec[i].u);
        if (i != vec.size() - 1) printf(",");
    }
    printf("]\n");
    printf("X = [");
    for (size_t i = 0; i < vec.size(); ++i) {
        printf("%.6f", vec[i].x);
        if (i != vec.size() - 1) printf(",");
    }
    printf("]\n");
}

void opcontrol() {
	std::vector<input_output> u_vs_x = fopdt_system_identification(200);
	print_vector(u_vs_x);
}
