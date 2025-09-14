#ifndef MOTION_PROFILER_HPP
#define MOTION_PROFILER_HPP

#include "api.h"
#include "system-identification.hpp"
#include "ff-velocity-controller.hpp"
#include "structs.hpp"

double calculate_t_accel(const double v_max, const double a_max);

double calculate_d_accel(const double v_max, const double a_max);

double calculate_d_coast(const double d_total, const double d_accel);

double calculate_t_coast(const double d_coast, const double d_max);

double oned_mp(const double a_max, const double t_accel, const double v_max, const double t_coast, const double t);

double oned_mp_accel(const double a_max, const double t_accel, const double t_coast, const double t);

void motor_angle_mp_test(const pros::Motor& test_motor, const ff_constants test_motor_constants, const double v_max, const double a_max, const double angle);

#endif // MOTION_PROFILER_HPP
