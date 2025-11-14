#include "motor-controller.hpp"
#include "helper-functions.hpp"
#include "structs.hpp"

void MotorController::move_motor_acceleration(const double& desired_acceleration) {
    double current_velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
    double voltage = motor_ff.compute_voltage(desired_acceleration, current_velocity);
	move_motor_voltage(voltage);
}

double MotorController::get_motor_max_accel(const int& direction) {
    double motor_velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
    double motor_max_acceleration = 0.0;
    if (motor_velocity == 0.0) {
        motor_max_acceleration = (motor_constants.max_voltage - motor_velocity * motor_constants.K_v - direction * motor_constants.K_s) / motor_constants.K_a * 0.9;
    }
    else {
        motor_max_acceleration = (motor_constants.max_voltage - motor_velocity * motor_constants.K_v - sign(motor_velocity) * motor_constants.K_s) / motor_constants.K_a * 0.9;
    }
    return motor_max_acceleration;
}

void MotorController::move_motor_voltage(const double& voltage) {
    motor.move_voltage(static_cast<int>(std::lround(voltage * 1000.0))); // volts to millivolts
}

void MotorController::move_motor_volts_time(const double& voltage, const int& time) {
    for (int i = 0; i < time / 10; i++) {
		move_motor_voltage(voltage);
        pros::delay(10);
    }
}

wheel_vel_bounds MotorController::get_motor_vel_bounds(const double& dt) {
    wheel_vel_bounds result{};
    result.min = (motor.get_actual_velocity() * 2.0 * M_PI / 60.0 - get_motor_max_accel(-1) * dt);
    result.max = (motor.get_actual_velocity() * 2.0 * M_PI / 60.0 + get_motor_max_accel(1) * dt);
    return result;
}

double MotorController::get_desired_motor_acceleration(const double& desired_motor_vel, const double& dt) {
    double desired_motor_acceleration = (desired_motor_vel - motor.get_actual_velocity() * 2.0 * M_PI / 60.0) / dt;
    return desired_motor_acceleration;
}

double MotorController::bound_desired_motor_velocity(const double& desired_velocity, const double& dt) {
    double velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
	double max_velocity_change = get_motor_max_accel(sign(desired_velocity)) * dt;
    double max_velocity = velocity + max_velocity_change;
    double min_velocity = velocity - max_velocity_change;
	double desired_velocity_bounded = std::clamp(desired_velocity, min_velocity, max_velocity);
    
	const double ZERO_DEADBAND_RAD_PER_S = 1.2 * motor_constants.K_s / motor_constants.K_v;

	if (std::abs(desired_velocity_bounded) < ZERO_DEADBAND_RAD_PER_S) {
		desired_velocity_bounded = 0.0;
	}
	return desired_velocity_bounded;
}
