/**
 * \file motion-profiler.cpp
 * @brief Implementation of triangular and trapezoidal motion profiles.
 *
 * Provides the piecewise equations that generate velocity profiles for a
 * given distance, honoring maximum acceleration and velocity constraints.
 * Used by higher-level code to command smooth linear and angular motions.
 */
#include "motion-profiler.hpp"

bool mp::profileFinished(float time) {
    return (time >= end_time);
}

float mp::velocity(float time) {
    float velocity;
    if (b < 0.f) {
        velocity = triangular_motion_profile(time);
    }
    else {
        velocity = trapezoidal_motion_profile(time);
    }
    if (distance > 0) {
        return velocity;
    }
    else {
        return -velocity;
    }
}

float mp::triangular_motion_profile(float time) {
    if (time < end_time / 2.f) {
        return (max_acceleration * time);
    }
    else {
        return (-max_acceleration * (time - end_time/2.f) + max_acceleration * end_time/2.f);
    }
}

float mp::trapezoidal_motion_profile(float time) {
    if (time < t_4) {
        return (max_acceleration * time);    
    }
    else {
        if (time <= t_5) {
            return(max_velocity);
        }
        else {
            return (-max_acceleration * (time - t_5) + max_velocity);
        }
    }
}
