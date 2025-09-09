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
class DCff {
private:
    const double K_a;
    const double K_v;
    const double K_s;
    static double sign(double x) {
        return (x > 0) - (x < 0);
    }
public:
    DCff(double K_a_, double K_v_, double K_s_) : K_a(K_a_), K_v(K_v_), K_s(K_s_) {}


    double compute_voltage(double alpha /*rad/s^2*/, double omega /*rad/s*/) const {
        double u = K_a * alpha + K_v * omega + K_s * sign(omega);
        return u;
    }


};
class drivetrain {
private:
    pros::Motor& m1;
    pros::Motor& m2;
    pros::Motor& o1;
    pros::Motor& o2;
    pros::Motor& m3;
    pros::Motor& m4;
public:
    drivetrain(pros::Motor& m1_, pros::Motor& m2_, pros::Motor& o1_, pros::Motor& o2_, pros::Motor& m3_, pros::Motor& m4_) : m1(m1_), m2(m2_), o1(o1_), o2(o2_), m3(m3_), m4(m4_) {}

};
void opcontrol() {
    // std::vector<input_output> u_vs_x = fopdt_system_identification(200);
    // print_vector(u_vs_x);
    pros::Motor m1(1, pros::v5::MotorGears::blue);
    pros::Motor m2(2, pros::v5::MotorGears::blue);
    pros::Motor o1(3, pros::v5::MotorGears::blue);
    pros::Motor o2(4, pros::v5::MotorGears::blue);
    pros::Motor m3(5, pros::v5::MotorGears::blue);
    pros::Motor m4(6, pros::v5::MotorGears::blue);
    drivetrain mecanum(m1, m2, o1, o2, m3, m4);
    DCff feedforward(0.00380743747817, 0.510097726504, 0.17033803784);
    double acceleration = 0.0;
    double prev_velocity = 0.0;
    std::vector<input_output> result;
    for (int i = 1; i < 101; i++) {
        input_output sample;
        sample.u = feedforward.compute_voltage(acceleration, prev_velocity);
        test_motor.move_voltage(sample.u * 1000);
        acceleration += 6.6283;
        prev_velocity = test_motor.get_actual_velocity() * 2.f * M_PI / 60.f;
        sample.x = prev_velocity;
        result.push_back(sample);
        pros::delay(10);
    }
    test_motor.move_voltage(0);
    print_vector(result);
}
