#ifndef MOTION_PROFILER_HPP
#define MOTION_PROFILER_HPP

#include "api.h"
#include "structs.hpp"

double calculate_t_accel(double v_max, double a_max);

double calculate_d_accel(double v_max, double a_max);

double calculate_d_coast(double d_total, double d_accel);

double calculate_t_coast(double d_coast, double d_max);

double oned_mp(double a_max, double t_accel, double v_max, double t_coast, double t);

#endif // MOTION_PROFILER_HPP
