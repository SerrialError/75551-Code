#ifndef HELPER_FUNCTIONS_HPP
#define HELPER_FUNCTIONS_HPP

#include "api.h"

constexpr static double sign(double x) {
        return (x > 0) - (x < 0);
}

constexpr static double DEG_TO_RAD_NORM(double deg_cw) {
    // Convert degrees to radians and flip sign (CW -> CCW is a sign change)
    double rad = -deg_cw * (M_PI / 180.0);

    // Normalize into [0, 2*M_PI)
    double two_pi = 2.0 * M_PI;
    rad = std::fmod(rad, two_pi);           // fmod may give negative result
    if (rad < 0.0) rad += two_pi;
    return rad;
}

#endif // HELPER_FUNCTIONS_HPP
