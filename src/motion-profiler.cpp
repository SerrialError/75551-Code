#include "motion-profiler.hpp"

void oned_motion_profiler(pros::Motor& test_motor, ff_constants test_motor_constants) {
    DCff feedforward(test_motor_constants);
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