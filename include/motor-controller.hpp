#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

/**
 * \file motor-controller.hpp
 * @brief High-level motor control abstraction.
 *
 * Declares the `MotorController` class, which wraps a PROS motor and provides
 * feedforward-based acceleration and velocity control, deadband handling, and
 * velocity bounding. It is used by both the drivetrain and intake subsystems.
 */

#include "api.h"
#include "structs.hpp"
#include "controllers/first-order-feedforward.hpp"
#include "controllers/pid.hpp"
#include "helper-functions.hpp"

/**
 * @brief Individual motor controller with feedforward control
 *
 * Provides high-level control for a single motor using feedforward control
 * based on motor constants. The controller can apply voltages, accelerations,
 * and velocities while respecting physical constraints and deadbands. It also
 * tracks motor data for system identification purposes.
 */
class MotorController {
private:
    FirstOrderFeedforward ff_controller;
	PID pid_controller;
    std::vector<input_output> data;
	const float timestep;
public:
    /**
     * @brief Constructs a motor controller with a motor and constants
     *
     * Initializes the controller with a reference to a PROS motor, feedforward
     * constants for control, and a name for identification. Creates an internal
     * feedforward controller using the provided constants.
     *
     * @param[in] motor_ Reference wrapper to the PROS motor to control
     * @param[in] motor_constants_ Feedforward constants (K_a, K_v, K_s, etc.)
     * @param[in] motor_name_ Name identifier for this motor (for debugging/logging)
     */
    MotorController(const std::reference_wrapper<pros::Motor>& motor_,
              const FirstOrderFeedforwardConstants& ff_constants_,
			  const PIDConstants& pid_constants_,
			  const float timestep_,
              std::string_view name_)
        : motor(motor_),
          ff_constants(ff_constants_),
		  pid_constants(pid_constants_),
          ff_controller(ff_constants_),
		  pid_controller(pid_constants_, timestep),
		  timestep(timestep_),
          name(name_)
    {}
    FirstOrderFeedforwardConstants ff_constants;
	PIDConstants pid_constants;

	/**
	 * @brief Prints the stored motor data vector
	 *
	 * Outputs the collected motor input-output data to the console. This is
	 * useful for debugging and system identification purposes.
	 */
	void print_vector();
    
    /**
     * @brief Updates the stored motor data with current measurements
     *
     * Records the current motor state (voltage input and velocity output)
     * into the motor_data vector. This data can be used for system identification
     * or analysis of motor performance.
     */
    void update_data();
    
    std::reference_wrapper<pros::Motor> motor;
    
	std::string_view name;

	/**
	 * @brief Applies a desired acceleration using feedforward control
	 *
	 * Calculates and applies the voltage needed to achieve a desired acceleration
	 * at the current motor velocity. Uses the feedforward controller to compute
	 * the required voltage. If the desired acceleration is zero and brake mode
	 * is hold, applies the brake instead of voltage.
	 *
	 * @param[in] desired_acceleration Desired acceleration in rad/s^2 with brake mode
	 */
	void move_acceleration(const motorVelocityType& desired_acceleration);

	/**
	 * @brief Bounds velocity to zero if within deadband range
	 *
	 * Sets the desired velocity to zero if it falls within the deadband range,
	 * which is calculated based on motor constants. This prevents small voltage
	 * commands that would be insufficient to overcome static friction, reducing
	 * motor jitter and power consumption.
	 *
	 * @param[in] desired_velocity Desired velocity in rad/s
	 *
	 * @return Bounded velocity (0 if in deadband, otherwise unchanged)
	 */
	float bound_velocity_to_deadband(float desired_velocity);

	/**
	 * @brief Calculates maximum achievable acceleration in a given direction
	 *
	 * Determines the maximum acceleration the motor can achieve based on current
	 * velocity, voltage limits, and motor constants. Accounts for direction of
	 * motion and whether the motor is reversing. The calculation uses the
	 * feedforward model to find the acceleration limit.
	 *
	 * @param[in] reverse True if calculating acceleration in reverse direction
	 * @param[in] direction Sign of desired motion (-1 or 1) for static friction calculation
	 *
	 * @return Maximum achievable acceleration in rad/s^2
	 */
	float get_max_accel(bool reverse, int direction);

	/**
	 * @brief Applies a voltage directly to the motor
	 *
	 * Sets the motor voltage, clamping it to the maximum voltage limit defined
	 * in motor constants. The voltage is converted from volts to millivolts
	 * for the PROS motor API. This is a low-level control method that bypasses
	 * velocity and acceleration limiting.
	 *
	 * @param[in] voltage Desired voltage in volts (will be clamped to max_voltage)
	 */
	void move_voltage(const float& voltage);

	/**
	 * @brief Applies a voltage for a specified duration
	 *
	 * Sets the motor voltage and maintains it for the given time period.
	 * The function loops in 10ms increments to maintain the voltage for
	 * the requested duration.
	 *
	 * @param[in] voltage Desired voltage in volts
	 * @param[in] time Duration to maintain voltage in milliseconds
	 */
	void move_volts_time(const float& voltage, const int& time);

	/**
	 * @brief Calculates achievable velocity bounds for the next time step
	 *
	 * Determines the minimum and maximum velocities the motor can achieve
	 * in the next time step based on current velocity and maximum acceleration
	 * limits. The bounds account for both forward and reverse acceleration
	 * capabilities.
	 *
	 * @return Velocity bounds (min, max) in rad/s
	 */
	wheel_vel_bounds get_vel_bounds();

	/**
	 * @brief Calculates desired acceleration from desired velocity
	 *
	 * Computes the acceleration needed to reach a desired velocity within
	 * the given time step. This is a simple calculation: (desired_vel - current_vel) / dt.
	 * Used for feedforward control to improve velocity tracking.
	 *
	 * @param[in] desired_motor_vels Target velocity in rad/s
	 * @param[in] dt Time step in seconds for acceleration calculation
	 *
	 * @return Desired acceleration in rad/s^2
	 */
	float get_desired_acceleration(const float& desired_motor_vels);

	/**
	 * @brief Bounds desired velocity to achievable limits
	 *
	 * Limits a desired velocity to values that can be achieved within the given
	 * time step, accounting for maximum acceleration constraints and current
	 * motor velocity. Also applies deadband filtering to prevent small commands.
	 * This prevents the controller from requesting velocities that would require
	 * impossible accelerations.
	 *
	 * @param[in] desired_velocity Target velocity in rad/s
	 * @param[in] dt Time step in seconds for calculating velocity change limits
	 *
	 * @return Bounded velocity that is achievable within the time step
	 */
	float bound_desired_velocity(const float& desired_velocity);

	float get_angular_velocity();
};

#endif // MOTOR_CONTROLLER_HPP