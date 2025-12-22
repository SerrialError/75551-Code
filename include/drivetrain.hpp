#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"
#include "structs.hpp"
#include "ff-velocity-controller.hpp"
#include "localization.hpp"
#include "helper-functions.hpp"
#include "motor-controller.hpp"
#include "motion-profiler.hpp"
#include "system-identification.hpp"

/**
 * @brief Main drivetrain class for holonomic robot control
 *
 * This class manages a six-wheel holonomic drivetrain with motor controllers,
 * localization, and motion profiling capabilities. It provides methods for
 * controlling the robot's movement using differential drive kinematics,
 * field-oriented control, and various motion profiles including Ramsete
 * path following.
 */
class drivetrain {
private:
	wheels<MotorController> motors;
	const double wheelbase_length;
	const double trackwidth_length;
	static constexpr double wheel_radius = 2.0 / 2.0 * 0.0254;
	static constexpr double b_gain = 2.0;
	static constexpr double decimal_of_max_velocity = 0.195;
	static constexpr double decimal_of_max_acceleration = 0.3;
	static constexpr double gear_ratio = 48.0/36.0;
	const double max_wheels_ang_vel;
	const double max_wheels_ang_vel_scaled;
	const double min_wheels_ang_accel;

public:
	/**
	 * @brief Constructs a drivetrain object with specified motors and parameters
	 *
	 * Initializes the drivetrain with six motors, physical dimensions, motor
	 * constants, and localization sensors. Calculates maximum velocities and
	 * accelerations based on motor characteristics and creates motion profilers
	 * for linear and angular movement.
	 *
	 * @param[in] motors_ Reference wrapper array of six PROS motor objects
	 * @param[in] wheelbase_length_ Distance between front and back wheels in meters
	 * @param[in] trackwidth_length_ Distance between left and right wheels in meters
	 * @param[in] motor_constants_ Feedforward constants for each motor (K_a, K_v, K_s)
	 * @param[in] linear_wheel_ Rotation sensor for measuring forward/backward movement
	 * @param[in] horizontal_wheel_ Rotation sensor for measuring sideways movement
	 * @param[in] imu_ Inertial measurement unit for orientation tracking
	 * @param[in] Pose_ Initial pose (x, y, theta) of the robot
	 */
	drivetrain(const wheels<std::reference_wrapper<pros::Motor>>& motors_,
			const double wheelbase_length_,
			const double trackwidth_length_,
			const wheels<ff_constants>& motor_constants_,
			pros::Rotation& linear_wheel_,
			pros::Rotation& horizontal_wheel_,
			pros::Imu& imu_,
			pose Pose_);
	const double max_robot_lin_vel_scaled;
	const double max_robot_ang_vel;
	const double max_robot_ang_vel_scaled;
	const double max_robot_ang_accel_scaled;
	Localization localization;
	mp LinearMP;
	mp AngularMP;
	/**
	 * @brief Calculates maximum angular velocity for a given acceleration
	 *
	 * Uses the feedforward motor model to determine the maximum angular velocity
	 * achievable when the motor is operating at a specific angular acceleration.
	 * The calculation accounts for voltage limits, acceleration torque, and
	 * static friction.
	 *
	 * @param[in] angular_acceleration Desired angular acceleration in rad/s^2
	 * @param[in] motor_constant Feedforward constants for the motor
	 *
	 * @return Maximum angular velocity in rad/s that can be achieved
	 */
	double angular_velocity(const double angular_acceleration, ff_constants motor_constant) {
		return ((motor_constant.max_voltage-(motor_constant.K_a * angular_acceleration)-motor_constant.K_s)/motor_constant.K_v);
	};

	/**
	 * @brief Calculates maximum angular acceleration for a given velocity
	 *
	 * Uses the feedforward motor model to determine the maximum angular acceleration
	 * achievable when the motor is operating at a specific angular velocity.
	 * The calculation accounts for voltage limits, velocity-dependent back-EMF,
	 * and static friction.
	 *
	 * @param[in] angular_velocity Current angular velocity in rad/s
	 * @param[in] motor_constant Feedforward constants for the motor
	 *
	 * @return Maximum angular acceleration in rad/s^2 that can be achieved
	 */
	double angular_acceleration(const double angular_velocity, ff_constants motor_constant) {
		return ((motor_constant.max_voltage-(motor_constant.K_v * angular_velocity)-motor_constant.K_s)/motor_constant.K_a);
	};

	pros::Controller master{pros::E_CONTROLLER_MASTER};

	/**
	 * @brief Applies brakes to all motors and stops movement
	 *
	 * Sets all six motors to brake mode and applies the brake, then sets
	 * all motor voltages to zero. This ensures the robot comes to a complete
	 * stop and holds position.
	 */
	void motor_brakes();

	/**
	 * @brief Calculates velocity bounds for each wheel based on desired robot velocity
	 *
	 * Takes a desired robot pose velocity and existing wheel velocity bounds,
	 * then calculates new bounds that account for the kinematic constraints
	 * of the holonomic drivetrain. This ensures the desired velocity is
	 * achievable given the current motor capabilities.
	 *
	 * @param[in] desired_vels Desired robot velocity in pose coordinates (x, y, theta)
	 * @param[in] bounds Current velocity bounds for each wheel
	 *
	 * @return Updated velocity bounds for each wheel that satisfy the desired velocity
	 */
	wheels<wheel_vel_bounds> calculate_wheel_vels_bounds(const pose& desired_vels, const wheels<wheel_vel_bounds>& bounds);

	/**
	 * @brief Calculates wheel velocities from desired robot pose velocity
	 *
	 * Converts a desired robot velocity in pose coordinates to individual wheel
	 * velocities using holonomic kinematics. Returns an optional result that
	 * is empty if the desired velocity is not achievable within the given bounds.
	 *
	 * @param[in] desired_vels Desired robot velocity in pose coordinates (x, y, theta)
	 * @param[in] bounds Velocity bounds for each wheel that must be satisfied
	 *
	 * @return Optional wheel velocities if achievable, empty otherwise
	 */
	std::optional<wheels<double>> calculate_wheel_vels(const pose& desired_vels, const wheels<wheel_vel_bounds>& bounds);

	/**
	 * @brief Sets voltage for all six motors simultaneously
	 *
	 * Applies the specified voltage to each motor in the drivetrain. The voltages
	 * are applied directly without any velocity or acceleration limiting, so this
	 * method should be used with caution.
	 *
	 * @param[in] wheel_voltages Voltage values for each of the six wheels in volts
	 */
	void move_motor_volts(const wheels<double>& wheel_voltages);

	/**
	 * @brief Sets voltage for all motors for a specified duration
	 *
	 * Applies the specified voltage to each motor and maintains it for the given
	 * time period. The function loops in 10ms increments to maintain the voltage
	 * for the requested duration.
	 *
	 * @param[in] wheel_voltages Voltage values for each of the six wheels in volts
	 * @param[in] time Duration to maintain voltages in milliseconds
	 */
	void move_motor_volts_time(const wheels<double>& wheel_voltages, const int time);

	/**
	 * @brief Controls the robot using field-oriented holonomic drive
	 *
	 * Implements field-oriented control where joystick inputs are interpreted
	 * relative to the field coordinate system rather than the robot's current
	 * orientation. This allows intuitive control regardless of robot heading.
	 *
	 * @param[in] dt Time step in seconds for control loop calculations
	 */
	void field_oriented_holonomic_control(const double dt);

	/**
	 * @brief Gets the standard angle representation of the robot's orientation
	 *
	 * Returns the robot's current angle normalized to a standard range, typically
	 * [-PI, PI] or [0, 2*PI] depending on the implementation.
	 *
	 * @return Standardized angle in radians
	 */
	double get_standard_angle(void);

	/**
	 * @brief Converts joystick input value to velocity
	 *
	 * Maps a joystick analog value (typically -127 to 127) to a corresponding
	 * velocity value. The mapping may be linear or use a curve for better control
	 * feel at low speeds.
	 *
	 * @param[in] joystick Joystick analog value, typically in range [-127, 127]
	 *
	 * @return Velocity value in appropriate units (m/s or rad/s)
	 */
	double joystick_to_vel(const double joystick);

	/**
	 * @brief Calculates achievable velocity bounds for each wheel
	 *
	 * Determines the minimum and maximum velocities that each wheel can achieve
	 * in the next time step based on current motor state and physical constraints.
	 * The bounds account for maximum acceleration limits and current velocity.
	 *
	 * @param[in] dt Time step in seconds for calculating velocity change limits
	 *
	 * @return Velocity bounds (min, max) for each of the six wheels in m/s
	 */
	wheels<wheel_vel_bounds> get_wheel_vel_bounds(const double dt);

	/**
	 * @brief Calculates desired motor accelerations from desired velocities
	 *
	 * Converts desired motor velocities to the accelerations needed to achieve
	 * them within the given time step. This is used for feedforward control
	 * to improve tracking performance.
	 *
	 * @param[in] desired_motor_vels Target velocities for each motor with brake mode
	 * @param[in] dt Time step in seconds for acceleration calculation
	 *
	 * @return Desired accelerations for each motor in rad/s^2 with brake mode
	 */
	wheels<motorVelocityType> get_wanted_motor_accels(const wheels<motorVelocityType>& desired_motor_vels, const double dt);

	/**
	 * @brief Implements tank drive control using joystick inputs
	 *
	 * Controls the robot using a tank drive scheme where the left joystick Y-axis
	 * controls linear velocity and the right joystick X-axis controls angular
	 * velocity. The function reads controller inputs, calculates desired robot
	 * velocities, converts them to motor velocities, and applies feedforward
	 * control with acceleration limiting.
	 *
	 * @param[in] dt Time step in seconds for control loop calculations
	 */
	void tank_drive_control(const double dt);

	/**
	 * @brief Converts differential drive velocities to individual motor velocities
	 *
	 * Takes a desired linear and angular velocity for the robot and calculates
	 * the corresponding velocity for each of the six wheels using holonomic
	 * kinematics. The calculation accounts for wheelbase and trackwidth dimensions
	 * to properly distribute the motion across all wheels.
	 *
	 * @param[in] robot_velocity Desired linear and angular velocities for the robot
	 *
	 * @return Motor velocities for each of the six wheels in rad/s with brake mode
	 */
	wheels<motorVelocityType> differential_vels_to_motor_vels(differentialVels robot_velocity);

	/**
	 * @brief Executes a sequence of differential drive velocity commands
	 *
	 * Applies a series of robot velocity commands, converting each to motor
	 * velocities and executing them with feedforward control. This is useful
	 * for following pre-planned paths or executing motion profiles.
	 *
	 * @param[in] robot_vels Vector of desired robot velocities to execute sequentially
	 * @param[in] dt Time step in seconds for each velocity command
	 */
	void move_differential_robot_vels(const std::vector<differentialVels>& robot_vels, const double dt);

	/**
	 * @brief Computes Ramsete controller output for path following
	 *
	 * Implements the Ramsete nonlinear controller for tracking desired poses
	 * and velocities. The controller computes corrected linear and angular
	 * velocities that account for pose errors, providing stable path following
	 * with guaranteed convergence. Errors are computed in the robot's local
	 * coordinate frame for better control performance.
	 *
	 * @param[in] wanted_pose Desired pose (x, y, theta) to track
	 * @param[in] wanted_vels Desired linear and angular velocities to achieve
	 *
	 * @return Corrected differential velocities that account for pose errors
	 */
	differentialVels ramsete(pose wanted_pose, differentialVels wanted_vels);

	/**
	 * @brief Executes velocity commands with Ramsete path following correction
	 *
	 * Applies a sequence of robot velocities while using Ramsete controller to
	 * correct for pose errors. For each desired velocity, the function computes
	 * the corresponding desired pose, applies Ramsete correction, and executes
	 * the corrected velocity. This provides accurate path following even with
	 * disturbances or modeling errors.
	 *
	 * @param[in] robot_vels Vector of desired robot velocities to execute
	 * @param[in] wanted_pose Vector of corresponding desired poses for Ramsete correction
	 * @param[in] dt Time step in seconds for each velocity command
	 */
	void move_differential_robot_vels_ramsete(std::vector<differentialVels> robot_vels, std::vector<pose>, const double dt);

	/**
	 * @brief Applies desired accelerations to all motors using feedforward control
	 *
	 * Takes desired accelerations for each motor and applies the corresponding
	 * voltages using feedforward control. The feedforward controller uses motor
	 * constants to calculate the voltage needed to achieve the desired acceleration
	 * at the current velocity.
	 *
	 * @param[in] motor_accelerations Desired accelerations for each motor with brake mode
	 */
	void move_motor_accelerations(const wheels<motorVelocityType>& motor_accelerations);

	/**
	 * @brief Bounds desired motor velocities to achievable limits
	 *
	 * Limits desired motor velocities to values that can be achieved within the
	 * given time step, accounting for maximum acceleration constraints and current
	 * motor velocities. This prevents the controller from requesting velocities
	 * that would require impossible accelerations.
	 *
	 * @param[in] desired_motor_velocities Target velocities for each motor in rad/s
	 * @param[in] dt Time step in seconds for calculating velocity change limits
	 *
	 * @return Bounded motor velocities that are achievable within the time step
	 */
	wheels<double> bound_desired_motor_velocities(const wheels<double>& desired_motor_velocities, const double dt);

	/**
	 * @brief Executes a linear motion profile to move a specified distance
	 *
	 * Moves the robot forward or backward a specified distance using a motion
	 * profile. The profile accelerates to maximum velocity, maintains it if
	 * needed, then decelerates to a stop. The function blocks until the motion
	 * is complete and applies brakes at the end.
	 *
	 * @param[in] distance Distance to travel in meters (positive = forward, negative = backward)
	 */
	void linear_mp(const double distance);
	
	/**
	 * @brief Executes an angular motion profile to rotate a specified angle
	 *
	 * Rotates the robot by a specified angle using a motion profile. The profile
	 * accelerates to maximum angular velocity, maintains it if needed, then
	 * decelerates to a stop. The function blocks until the rotation is complete
	 * and applies brakes at the end.
	 *
	 * @param[in] angle Angle to rotate in radians (positive = counterclockwise, negative = clockwise)
	 */
	void angular_mp(const double angle);
	
	/**
	 * @brief Moves to a target pose using motion profiles
	 *
	 * Moves the robot to a desired pose by first rotating to face the target,
	 * then moving linearly to the target position. Uses separate motion profiles
	 * for rotation and translation. This is a simple approach that does not account
	 * for simultaneous rotation and translation.
	 *
	 * @param[in] desired_pose Target pose (x, y, theta) to reach
	 */
	void mtp_mp(const pose desired_pose);
		
	/**
	 * @brief Moves to a target pose using Ramsete path following
	 *
	 * Moves the robot to a desired pose using Ramsete controller for path following.
	 * This method provides smoother and more accurate motion than the simple
	 * motion profile approach by allowing simultaneous rotation and translation
	 * with pose error correction.
	 *
	 * @param[in] desired_pose Target pose (x, y, theta) to reach
	 */
	void mtp_mp_ramsete(const pose desired_pose);

	/**
	 * @brief Calculates and prints feedforward constants for all motors
	 *
	 * Performs system identification on all six motors to determine their
	 * feedforward constants (K_v, K_a, K_s). The results are printed to the
	 * console for use in motor control. This function blocks while performing
	 * the identification process.
	 */
	void calculate_and_print_motor_constants();
};

#endif // DRIVETRAIN_HPP
