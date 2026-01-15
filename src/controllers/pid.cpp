#include "controllers/pid.hpp"

float PID::compute(const float& setpoint, const float& output) {
    float error = setpoint - output;
	// integral separation to get rid of steady state error only if position state controller
	if (constants.state == PIDState::position) {
		if (fabs(error) < constants.integral_start) {
    		error_integral += error * timestep;
		}
		else {
			error_integral = 0.f;
		}
	}
	else {
    	error_integral += error * timestep;
	}
	// prevents integral windup
	error_integral = std::clamp(error_integral, -constants.max_integral, constants.max_integral);
	// integral reset if overshoot only for position state controller
	if ((constants.state == PIDState::position) && sign(error) != sign(previous_error)) {
		error_integral = 0.f;
	}
	// derivative based on output instead of error to prevent derivative kick
	float output_derivative = (output - previous_output) / timestep;
    previous_output = output;
	previous_error = error;
    return constants.Kp * error + constants.Ki * error_integral - constants.Kd * output_derivative;
}

void PID::reset(const float setpoint, const float output) {
	error_integral = 0.f;
	previous_output = output;
	previous_error = setpoint - output;
}
