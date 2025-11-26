// structs.hpp
#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include "api.h"

template<typename T>
struct wheels {
    T m1, m2, o1, o2, m3, m4;

    // non-const view as array of reference_wrapper
    auto asArray() {
        return std::array<std::reference_wrapper<T>, 6>{
            std::ref(m1), std::ref(m2), std::ref(o1),
            std::ref(o2), std::ref(m3), std::ref(m4)
        };
    }

    // const-view: reference to const T
    auto asArray() const {
        return std::array<std::reference_wrapper<const T>, 6>{
            std::cref(m1), std::cref(m2), std::cref(o1),
            std::cref(o2), std::cref(m3), std::cref(m4)
        };
    }

    // convenient index access (non-const)
    T& operator[](size_t i) {
        switch (i) {
            case 0: return m1;
            case 1: return m2;
            case 2: return o1;
            case 3: return o2;
            case 4: return m3;
            default: return m4;
        }
    }

    // index access const
    const T& operator[](size_t i) const {
        switch (i) {
            case 0: return m1;
            case 1: return m2;
            case 2: return o1;
            case 3: return o2;
            case 4: return m3;
            default: return m4;
        }
    }
};

template<typename T>
struct rollers {
    T fb; // front bottom
    T ft; // front top
    T bb; // back bottom
    T bt; // back top
    // non-const view as array of reference_wrapper
    auto asArray() {
        return std::array<std::reference_wrapper<T>, 4>{
            std::ref(fb), std::ref(ft), std::ref(bb), std::ref(bt)
        };
    }

    // const-view: reference to const T
    auto asArray() const {
        return std::array<std::reference_wrapper<const T>, 4>{
            std::cref(fb), std::cref(ft), std::cref(bb), std::cref(bt)
        };
    }

    // convenient index access (non-const)
    T& operator[](size_t i) {
        switch (i) {
            case 0: return fb;
            case 1: return ft;
            case 2: return bb;
            default: return bt;
        }
    }

    // index access const
    const T& operator[](size_t i) const {
        switch (i) {
            case 0: return fb;
            case 1: return ft;
            case 2: return bb;
            default: return bt;
        }
    }
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
    double max_ang_vel;
    double max_voltage;
};

struct wheel_vel_bounds {
    double min, max;
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

enum class motorStateType {
    forward,
    reverse,
    hold,
    off
};

enum color {
    red,
    blue,
	none
};

struct motorVelocityType {
    double velocity;
    pros::motor_brake_mode_e  brakeMode;
};

struct differentialVels {
    double linear;
    double angular;
};

enum autons {
    blueRight,
    blueLeft,
    redRight,
    redLeft
};

#endif // STRUCTS_HPP
