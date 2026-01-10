/**
 * \file motor-controller.cpp
 * @brief Implementation of feedforward-based motor control primitives.
 *
 * Implements methods that translate desired accelerations and velocities
 * into constrained voltage commands, while enforcing acceleration limits
 * and applying a small deadband around zero to avoid jitter.
 */
#include "motor-controller.hpp"
#include "helper-functions.hpp"
#include "structs.hpp"

void MotorController::move_motor_acceleration(const motorVelocityType& desired_acceleration) {
    // Convert measured speed from RPM to rad/s for feedforward computation.
    float current_velocity = motor.get().get_actual_velocity() * 2.0 * static_cast<float>(static_cast<float>(M_PI)) / 60.0;
    float voltage = ff_controller.compute({desired_acceleration.velocity, current_velocity});
    if (voltage == 0.0 && desired_acceleration.brakeMode == pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD) {
        motor.get().set_brake_mode(desired_acceleration.brakeMode);
        motor.get().brake();
    }
    else {
		move_motor_voltage(voltage);
    }
}

float MotorController::get_motor_max_accel(bool reverse, int direction) {
    float motor_velocity = motor.get().get_actual_velocity() * 2.0 * static_cast<float>(static_cast<float>(M_PI)) / 60.0;
    float motor_max_acceleration = 0.0;
    if (!reverse) {
        if (motor_velocity == 0.0) {
            motor_max_acceleration = (ff_constants.max_voltage - motor_velocity * ff_constants.Kv - direction * ff_constants.Ks) / ff_constants.Ka;
        }
        else {
            motor_max_acceleration = (ff_constants.max_voltage - motor_velocity * ff_constants.Kv - sign(motor_velocity) * ff_constants.Ks) / ff_constants.Ka;
        }
    }
    else {
        if (motor_velocity == 0.0) {
            motor_max_acceleration = (-1.0 * ff_constants.max_voltage - motor_velocity * ff_constants.Kv - direction * ff_constants.Ks) / ff_constants.Ka;
        }
        else {
            motor_max_acceleration = (-1.0 * ff_constants.max_voltage - motor_velocity * ff_constants.Kv - sign(motor_velocity) * ff_constants.Ks) / ff_constants.Ka;
        }
    }

    return motor_max_acceleration;
}

void MotorController::move_motor_voltage(const float& voltage) {
    float clamped_voltage = std::clamp(voltage, -ff_constants.max_voltage, ff_constants.max_voltage);
    motor.get().move_voltage(static_cast<int>(std::lround(voltage * 1000.0))); // volts to millivolts
}

void MotorController::move_motor_volts_time(const float& voltage, const int& time) {
    for (int i = 0; i < time / 10; i++) {
		move_motor_voltage(voltage);
        pros::delay(10);
    }
}

wheel_vel_bounds MotorController::get_motor_vel_bounds() {
    wheel_vel_bounds result{};
    result.min = (motor.get().get_actual_velocity() * 2.0 * static_cast<float>(static_cast<float>(M_PI)) / 60.0 + get_motor_max_accel(true, -1) * timestep);
    result.max = (motor.get().get_actual_velocity() * 2.0 * static_cast<float>(static_cast<float>(M_PI)) / 60.0 + get_motor_max_accel(false, 1) * timestep);
    return result;
}

float MotorController::get_desired_motor_acceleration(const float& desired_motor_vel) {
    float desired_motor_acceleration = (desired_motor_vel - motor.get().get_actual_velocity() * 2.0 * static_cast<float>(static_cast<float>(M_PI)) / 60.0) / timestep;
    return desired_motor_acceleration;
}

float MotorController::bound_velocity_to_deadband(float desired_velocity) {
	const float ZERO_DEADBAND_RAD_PER_S = 1.2 * ff_constants.Ks / ff_constants.Kv;
	if (std::abs(desired_velocity) < ZERO_DEADBAND_RAD_PER_S) {
		desired_velocity = 0.0;
	}
	return desired_velocity;
}

float MotorController::bound_desired_motor_velocity(const float& desired_velocity) {
    float velocity = motor.get().get_actual_velocity() * 2.0 * static_cast<float>(static_cast<float>(M_PI)) / 60.0;
	float max_velocity_change = get_motor_max_accel(false, sign(desired_velocity)) * timestep;
    float min_velocity_change = get_motor_max_accel(true, sign(desired_velocity)) * timestep;
    float max_velocity = velocity + max_velocity_change;
    float min_velocity = velocity + min_velocity_change;
	float desired_velocity_bounded = bound_velocity_to_deadband(std::clamp(desired_velocity, min_velocity, max_velocity));
	return desired_velocity_bounded;
}
