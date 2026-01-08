/**
 * \file ff-velocity-controller.cpp
 * @brief Stub translation unit for the feedforward controller.
 *
 * Currently only includes the header so that `DCff` is compiled as part
 * of the project; all logic is implemented inline in the header.
 */
#include "controllers/pid.hpp"

double PID::compute(const double& setpoint, const double& output) {
    double error = setpoint - output;
    error_integral += error * timestep;
    double error_derivative = (error - previous_error) / timestep;
    previous_error = error;
    return constants.Kp * error + constants.Ki * error_integral + constants.Kd * error_derivative;
}