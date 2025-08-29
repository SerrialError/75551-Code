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

struct kappa_e_type {
	double voltage;
	double angular_velocity;
};

struct kappa_tau_type {
	double current;
	double torque;
};

std::vector<kappa_e_type> velocity_constant(int n, int settle_ms) {
	std::vector<kappa_e_type> result;
	for (int i = 1; i < 13; i++) {
		test_motor.move_voltage(1000*i);
		pros::delay(settle_ms);
		double voltage_total{0.0};
		double velocity_total{0.0};
		for (int j = 0; j < n; j++) {
			voltage_total += test_motor.get_voltage() / 1000.f;
			velocity_total += test_motor.get_actual_velocity() * 2.f * M_PI / 60.f;
			pros::delay(10);
		};
		double avg_voltage = voltage_total / n;
		double avg_velocity = velocity_total / n;
		result.push_back({avg_voltage, avg_velocity}); 
	};
	test_motor.move_voltage(0);

	return result;	
};

std::vector<kappa_tau_type> torque_constant(int n, int settle_ms) {
	std::vector<kappa_tau_type> result;
	for (int i = 1; i < 13; i++) {
		test_motor.move_voltage(1000*i);
		pros::delay(settle_ms);
		double current_total{0.0};
		double torque_total{0.0};
		for (int j = 0; j < n; j++) {
			current_total += test_motor.get_current_draw() / 1000.f;
			torque_total += test_motor.get_torque();
			pros::delay(10);
		};
		double avg_current = current_total / n;
		double avg_torque = torque_total / n;
		result.push_back({avg_current, avg_torque}); 
	};
	test_motor.move_voltage(0);

	return result;	
};

void print_vector(const std::vector<kappa_e_type>& vec) {
    printf("{");
    for (size_t i = 0; i < vec.size(); ++i) {
        printf("(%.2f,%.2f)", vec[i].voltage, vec[i].angular_velocity);
        if (i != vec.size() - 1) printf(",");
    }
    printf("}\n");
}

void print_vector(const std::vector<kappa_tau_type>& vec) {
    printf("{");
    for (size_t i = 0; i < vec.size(); ++i) {
        printf("(%.2f,%.2f)", vec[i].current, vec[i].torque);
        if (i != vec.size() - 1) printf(",");
    }
    printf("}\n");
}

void opcontrol() {
	int n = 500;
	std::vector<kappa_e_type> V_vs_w = velocity_constant(n, 800);
	std::vector<kappa_tau_type> I_vs_tau = torque_constant(n, 800);
	print_vector(V_vs_w);
	print_vector(I_vs_tau);
}
