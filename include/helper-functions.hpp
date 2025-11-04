#ifndef HELPER_FUNCTIONS_HPP
#define HELPER_FUNCTIONS_HPP

#include "api.h"

constexpr static double sign(double x) {
        return (x > 0) - (x < 0);
}

constexpr static double wrapToPi(double a) {
    double r = std::fmod(a + M_PI, 2.0*M_PI);
    if (r < 0) r += 2.0*M_PI;
    return r - M_PI;
}

constexpr static double DEG_TO_RAD_NORM(double deg_cw) {
    // Convert degrees to radians and flip sign (CW -> CCW is a sign change)
    double rad = -deg_cw * (M_PI / 180.0);

    // Normalize into [-PI, PI)
    rad = wrapToPi(rad);
    return rad;
}

#endif // HELPER_FUNCTIONS_HPP
