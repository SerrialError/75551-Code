// structs.hpp
#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include "api.h"

struct input_output {
    double u;
    double x;
};

struct wheel_vels {
    double m1_vel;
    double m2_vel;
    double o1_vel;
    double o2_vel;
    double m3_vel;
    double m4_vel;
};

struct pose_vels {
    double x_vel;
    double y_vel;
    double theta_vel;
};

struct wheel_vel_lim {
    double min;
    double max;
};

struct wheel_vel_lims {
    wheel_vel_lim m1_limits;
    wheel_vel_lim m2_limits;
    wheel_vel_lim o1_limits;
    wheel_vel_lim o2_limits;
    wheel_vel_lim m3_limits;
    wheel_vel_lim m4_limits;
};

struct ff_constants {
    const double K_a;
    const double K_v;
    const double K_s;
};

#endif // STRUCTS_HPP
