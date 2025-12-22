#ifndef HELPER_FUNCTIONS_HPP
#define HELPER_FUNCTIONS_HPP

/**
 * \file helper-functions.hpp
 * @brief Small math helpers used across motion and control code.
 *
 * Declares utility functions such as `sign`, `wrapToPi`, `DEG_TO_RAD_NORM`,
 * and `sinc` that are used by drivetrain control, localization, and
 * motion profiling. These helpers encapsulate common mathematical
 * operations on angles and scalar values.
 */

#include "api.h"

/**
 * @brief Returns the sign of a number
 *
 * Computes the sign function: returns 1 for positive values, -1 for negative
 * values, and 0 for zero. This is useful for determining direction in control
 * algorithms, particularly for handling static friction.
 *
 * @param[in] x Input value
 *
 * @return 1 if x > 0, -1 if x < 0, 0 if x == 0
 */
constexpr static double sign(double x) {
        return (x > 0) - (x < 0);
}

/**
 * @brief Wraps an angle to the range [-PI, PI)
 *
 * Normalizes an angle in radians to the standard range [-PI, PI) by adding
 * or subtracting multiples of 2*PI. This is useful for angle arithmetic
 * where angles may accumulate beyond the standard range.
 *
 * @param[in] a Angle in radians to wrap
 *
 * @return Angle normalized to [-PI, PI) range
 */
constexpr static double wrapToPi(double a) {
    double r = std::fmod(a + M_PI, 2.0*M_PI);
    if (r < 0) r += 2.0*M_PI;
    return r - M_PI;
}

/**
 * @brief Converts degrees to radians and normalizes to [-PI, PI)
 *
 * Converts an angle from degrees to radians, flips the sign (clockwise becomes
 * counterclockwise), and normalizes the result to the range [-PI, PI). This
 * is specifically designed for PROS rotation sensors which report clockwise
 * angles in degrees.
 *
 * @param[in] deg_cw Angle in degrees, measured clockwise from reference
 *
 * @return Angle in radians, normalized to [-PI, PI), with counterclockwise positive
 */
constexpr static double DEG_TO_RAD_NORM(double deg_cw) {
    // Convert degrees to radians and flip sign (CW -> CCW is a sign change)
    double rad = -deg_cw * (M_PI / 180.0);

    // Normalize into [-PI, PI)
    rad = wrapToPi(rad);
    return rad;
}

/**
 * @brief Computes the sinc function with special handling at zero
 *
 * Calculates sin(x)/x, which is the sinc function. At x=0, the function
 * is defined as 1 (the limit as x approaches 0). For numerical stability,
 * values very close to zero (within 1e-5) are treated as exactly zero.
 * This function is used in the Ramsete controller for smooth error correction.
 *
 * @param[in] x Input value
 *
 * @return sin(x)/x if |x| >= 1e-5, 1.0 otherwise
 */
constexpr static double sinc(double x) {
    return (std::abs(x) < 1e-5f) ? 1.0f : std::sin(x) / x;
}

#endif // HELPER_FUNCTIONS_HPP
