// structs.hpp
#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include "api.h"

template<typename T>
struct wheels {
    T m1;
    T m2;
    T o1;
    T o2;
    T m3;
    T m4;
};

template<typename T>
struct rollers {
    T fb; // front bottom
    T ft; // front top
    T bb; // back bottom
    T bt; // back top
};

struct pose {
    double x;
    double y;
    double theta;
};

struct ff_constants {
    double K_a;
    double K_v;
    double K_s;
    double max_ang_accel;
    double max_ang_vel;
};

struct wheel_vel_lim {
    double min;
    double max;
};

struct input_output {
    double u;
    double x;
};

enum rollerStateType {
    intakeOff,
    intakeOnly,
    bottomScore,
    midScore,
    topScore
};

enum motorStateType {
    forward,
    reverse,
    hold,
    off
};

#endif // STRUCTS_HPP
