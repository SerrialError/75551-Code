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
