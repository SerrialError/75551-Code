#ifndef MOTION_PROFILER_HPP
#define MOTION_PROFILER_HPP

#include "api.h"
#include "system-identification.hpp"
#include "ff-velocity-controller.hpp"
#include "structs.hpp"

double calculate_t_accel(double v_max, double a_max);

double calculate_d_accel(double v_max, double a_max);

double calculate_d_coast(double d_total, double d_accel);

double calculate_t_coast(double d_coast, double d_max);

double oned_mp(double a_max, double t_accel, double v_max, double t_coast, double t);

double oned_mp_accel(double a_max, double t_accel, double t_coast, double t);

void motor_angle_mp_test(pros::Motor& test_motor, ff_constants test_motor_constants, double v_max, double a_max, double angle);

#endif // MOTION_PROFILER_HPP