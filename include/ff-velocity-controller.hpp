#ifndef FF_VELOCITY_CONTROLLER_HPP
#define FF_VELOCITY_CONTROLLER_HPP

/**
 * \file ff-velocity-controller.hpp
 * @brief Feedforward velocity controller interfaces.
 *
 * Declares the `DCff` feedforward controller class for DC motors and a
 * placeholder `pd` controller. These classes are used to compute motor
 * voltages from desired accelerations and velocities using identified
 * motor constants.
 */

#include "api.h"
#include "structs.hpp"
#include "helper-functions.hpp"

/**
 * @brief DC motor feedforward controller
 *
 * Implements a feedforward control law for DC motors based on the motor's
 * physical characteristics. The controller calculates the voltage needed to
 * achieve a desired acceleration and velocity, accounting for back-EMF,
 * acceleration torque, and static friction.
 */
class DCff {
private:
    ff_constants constants;
public:
    /**
     * @brief Constructs a feedforward controller with motor constants
     *
     * Initializes the controller with feedforward constants that characterize
     * the motor's response. These constants are typically obtained through
     * system identification.
     *
     * @param[in] constants_ Feedforward constants (K_a, K_v, K_s, max_ang_vel, max_voltage)
     */
    DCff(ff_constants constants_) : constants(constants_) {}
    
    /**
     * @brief Computes the voltage needed to achieve desired acceleration and velocity
     *
     * Calculates the control voltage using the feedforward model:
     * u = K_a * alpha + K_v * omega + K_s * sign(omega)
     * where K_a accounts for acceleration torque, K_v for back-EMF, and
     * K_s for static friction. The sign function ensures friction opposes motion.
     *
     * @param[in] alpha Desired angular acceleration in rad/s^2
     * @param[in] omega Current angular velocity in rad/s
     *
     * @return Required voltage in volts to achieve the desired acceleration
     */
    double compute_voltage(double alpha /*rad/s^2*/, double omega /*rad/s*/) const {
        double u = constants.K_a * alpha + constants.K_v * omega + constants.K_s * sign(omega);
        return u;
    }
};

/**
 * @brief Proportional-Derivative (PD) controller class
 *
 * A PD controller for velocity control that uses proportional and derivative
 * terms to reduce error between desired and measured velocity. This class
 * is currently incomplete and not fully implemented.
 */
class pd {
private:
    double r; //setpoint(desired velocity)
    double u; //control input
    double e; //error(aceleration)
    double y; //output(measured velocity)


public:

};
#endif // ff-velocity-controller.hpp
