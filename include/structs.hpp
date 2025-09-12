// structs.hpp
#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include "api.h"

struct input_output {
    double u;
    double x;
};

struct wheels {
    double m1;
    double m2;
    double o1;
    double o2;
    double m3;
    double m4;
};

struct pose {
    double x;
    double y;
    double theta;
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
