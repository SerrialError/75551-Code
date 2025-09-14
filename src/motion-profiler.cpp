#include "motion-profiler.hpp"

double calculate_t_accel(const double v_max, const double a_max) {
    double t_accel = v_max / a_max;
    return t_accel;
}

double calculate_d_accel(const double v_max, const double a_max) {
    double d_accel = pow(v_max,2) / (2 * a_max);
    return d_accel;
}

double calculate_d_coast(const double d_total, const double d_accel) {
    double d_coast = d_total - 2 * d_accel;
    return d_coast;
}

double calculate_t_coast(const double d_coast, const double v_max) {
    double t_coast = d_coast / v_max;
    return t_coast;
}

double oned_mp(const double a_max, const double t_accel, const double v_max, const double t_coast, const double t) {
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

double oned_mp_accel(const double a_max, const double t_accel, const double t_coast, const double t) {
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

void motor_angle_mp_test(const pros::Motor& test_motor, const ff_constants test_motor_constants, const double v_max, const double a_max, const double angle) {
    const double t_accel = calculate_t_accel(v_max, a_max);
    const double d_accel = calculate_d_accel(v_max, a_max);
    const double d_coast = calculate_d_coast(angle, d_accel);
    const double t_coast = calculate_t_coast(d_coast, v_max);
    const int total_t = static_cast<int>((t_accel * 2.f + t_coast) * 100.f); // 10 ms -> 1s
    DCff feedforward(test_motor_constants);
    std::vector<input_output> result;
    for (int i = 1; i < total_t + 1; i++) {
        input_output sample;
        double acceleration = oned_mp_accel(a_max, t_accel, t_coast, i * 100.f);
        double prev_velocity = test_motor.get_actual_velocity() * 2.f * M_PI / 60.f;
        double voltage = feedforward.compute_voltage(acceleration, prev_velocity);
        sample.u = i * 1.f;
        test_motor.move_voltage(voltage * 1000);
        sample.x = prev_velocity;
        result.push_back(sample);
        pros::delay(10);
    }
    test_motor.move_voltage(0);
    print_vector(result);
}
