#include "motion-profiler.hpp"

double calculate_t_accel(double v_max, double a_max) {
    double t_accel = v_max / a_max;
    return t_accel;
}

double calculate_d_accel(double v_max, double a_max) {
    double d_accel = pow(v_max,2) / (2 * a_max);
    return d_accel;
}

double calculate_d_coast(double d_total, double d_accel) {
    double d_coast = d_total - 2 * d_accel;
    return d_coast;
}

double calculate_t_coast(double d_coast, double v_max) {
    double t_coast = d_coast / v_max;
    return t_coast;
}

double oned_mp(double a_max, double t_accel, double v_max, double t_coast, double t) {
    double v{};
    if (t < t_accel) {
        v = a_max * t;
    }
    else if (t < t_accel + t_coast) {
        v = v_max;
    }
    else {
        v = v_max - a_max * (t - t_accel - t_coast);
    }
    return v;
}

double oned_mp_accel(double a_max, double t_accel, double t_coast, double t) {
    double a{};
    if (t < t_accel) {
        a = a_max;
    }
    else if (t <= t_accel + t_coast) {
        a = 0;
    }
    else {
        a = - a_max;
    }
    return a;
}

void motor_angle_mp_test(pros::Motor& test_motor, ff_constants test_motor_constants, double v_max, double a_max, double angle) {
    double t_accel = calculate_t_accel(v_max, a_max);
    double d_accel = calculate_d_accel(v_max, a_max);
    double d_coast = calculate_d_coast(angle, d_accel);
    double t_coast = calculate_t_coast(d_coast, v_max);
    DCff feedforward(test_motor_constants);
    std::vector<input_output> result;
    for (int i = 1; i < (t_accel * 2 + t_coast) * 100 + 1; i++) {
        input_output sample;
        double acceleration = oned_mp_accel(a_max, t_accel, t_coast, t*.001;
        double prev_velocity = test_motor.get_actual_velocity() * 2.f * M_PI / 60.f;
        sample.u = feedforward.compute_voltage(acceleration, prev_velocity);
        test_motor.move_voltage(sample.u * 1000);
        sample.x = prev_velocity;
        result.push_back(sample);
        pros::delay(10);
    }
    test_motor.move_voltage(0);
    print_vector(result);
}