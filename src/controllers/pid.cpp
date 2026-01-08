/**
 * \file ff-velocity-controller.cpp
 * @brief Stub translation unit for the feedforward controller.
 *
 * Currently only includes the header so that `DCff` is compiled as part
 * of the project; all logic is implemented inline in the header.
 */
#include "controllers/pid.hpp"

float PID::compute(const float& setpoint, const float& output) {
    float error = setpoint - output;
    error_integral += error * timestep;
    float error_derivative = (error - previous_error) / timestep;
    previous_error = error;
    return constants.Kp * error + constants.Ki * error_integral + constants.Kd * error_derivative;
}
