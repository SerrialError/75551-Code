#ifndef INTAKE_HPP
#define INTAKE_HPP

/**
 * \file intake.hpp
 * @brief Intake mechanism control and color sorting logic.
 *
 * Declares the `intake` class, which manages a two-roller intake mechanism
 * using `MotorController` objects. The intake supports multiple operating
 * states and can optionally perform alliance-based color sorting.
 */

#include "api.h"
#include "structs.hpp"
#include "controllers/first-order-feedforward.hpp"
#include "motor-controller.hpp"
#include "helper-functions.hpp"

/**
 * @brief Intake system controller with color sorting capabilities
 *
 * Manages a two-roller intake system with motor controllers and optional
 * color-based sorting. The intake can operate in various states (off, intake,
 * scoring at different heights) and can automatically sort game pieces by
 * color when enabled.
 */
class intake {
private:
    pros::Controller master{pros::E_CONTROLLER_MASTER};
    rollers<MotorController> motors;
    motorStateType motorState;
	const float redHue = 4.0;
	const float blueHue = 200.0;
	const float redHueUncertainty = 7.0;
    const float blueHueUncertainty = 20.0;

public:
    /**
     * @brief Constructs an intake system with two motors
     *
     * Initializes the intake with front and back roller motors, each with
     * their own motor controller and feedforward constants. The motors are
     * named "front" and "back" for identification purposes.
     *
     * @param[in] motors_ Reference wrapper array of two PROS motor objects (front, back)
     * @param[in] motor_constants_ Feedforward constants for each motor
     */
    intake(const rollers<std::reference_wrapper<pros::Motor>>& motors_,
           const rollers<FirstOrderFeedforwardConstants>& motor_ff_constants_,
		   const rollers<PIDConstants>& motor_pid_constants_,
		   const float timestep)
        : motors{ MotorController(motors_.front, motor_ff_constants_.front, motor_pid_constants_.front, timestep, "front"),
                  MotorController(motors_.back, motor_ff_constants_.back, motor_pid_constants_.front, timestep, "back") }
    {}
    color allianceColor = red;
    rollerStateType intakeState = intakeOff;
	bool colorSorting = false;
	/**
	 * @brief Applies desired accelerations to both intake motors
	 *
	 * Takes desired accelerations for the front and back rollers and applies
	 * the corresponding voltages using feedforward control. This allows
	 * precise control of intake speed and acceleration.
	 *
	 * @param[in] motor_accelerations Desired accelerations for front and back rollers with brake mode
	 */
	void move_motor_accelerations(const rollers<motorVelocityType>& motor_accelerations);
	
	/**
	 * @brief Calculates desired motor accelerations from desired velocities
	 *
	 * Converts desired roller velocities to the accelerations needed to achieve
	 * them within the given time step. This is used for feedforward control
	 * to improve velocity tracking performance.
	 *
	 * @param[in] desired_motor_vels Target velocities for each roller with brake mode
	 * @param[in] dt Time step in seconds for acceleration calculation
	 *
	 * @return Desired accelerations for each roller in rad/s^2 with brake mode
	 */
	rollers<motorVelocityType> get_wanted_motor_accels(const rollers<motorVelocityType>& desired_motor_vels);

	/**
	 * @brief Gets the motor states corresponding to the current intake state
	 *
	 * Maps the current intake state (intakeOff, intakeOnly, bottomScore, etc.)
	 * to the desired motor states for the front and back rollers. Each state
	 * defines specific velocity and brake mode requirements for each roller.
	 *
	 * @return Motor states (velocity or special state) for front and back rollers
	 */
	rollers<motorStateType> get_roller_states(void);

	/**
	 * @brief Converts a roller state to a motor velocity command
	 *
	 * Takes a desired roller state (which can be a velocity scale factor or
	 * a special state like hold/off) and converts it to a motor velocity
	 * command with appropriate brake mode. The velocity is scaled by the
	 * motor's maximum angular velocity and bounded to the deadband.
	 *
	 * @param[in] wanted_roller_state Desired state (velocity scale or special state)
	 * @param[in] motor Motor controller to get constants and apply deadband
	 *
	 * @return Motor velocity command with brake mode
	 */
	motorVelocityType get_desired_motor_state(motorStateType wanted_roller_state, MotorController motor);

	/**
	 * @brief Converts roller states to motor velocity commands for both rollers
	 *
	 * Takes desired states for both rollers and converts them to motor velocity
	 * commands. This is a convenience function that applies get_desired_motor_state
	 * to both the front and back rollers.
	 *
	 * @param[in] wanted_roller_states Desired states for front and back rollers
	 *
	 * @return Motor velocity commands with brake modes for both rollers
	 */
	rollers<motorVelocityType> get_desired_motor_states(rollers<motorStateType> wanted_roller_states);

	/**
	 * @brief Updates intake state based on detected block color
	 *
	 * If color sorting is enabled and the intake is active, checks the color
	 * of the detected block and adjusts the intake state accordingly. If the
	 * block is the wrong color, switches between midScore and topScore states
	 * to route the block appropriately.
	 */
	void get_intake_state();

	/**
	 * @brief Updates the intake system state and applies motor commands
	 *
	 * Main update function that should be called periodically. If color sorting
	 * is enabled, it updates the intake state based on detected block color.
	 * Then it calculates desired motor states, converts them to accelerations,
	 * and applies them to the motors.
	 *
	 * @param[in] dt Time step in seconds for control loop calculations
	 */
	void update_intake_state(const float& dt);

	/**
	 * @brief Checks if the detected block color matches the alliance color
	 *
	 * Determines whether the currently detected block is the correct color
	 * for the robot's alliance. Returns true if the block matches the alliance
	 * color or if no block is detected (none color).
	 *
	 * @return True if block color matches alliance or no block detected, false otherwise
	 */
	bool is_correct_color(void);

	/**
	 * @brief Detects the color of a block in the intake
	 *
	 * Uses an optical sensor to measure the hue of a block and determines
	 * its color (red, blue, or none) based on predefined hue values and
	 * uncertainty thresholds. The function accounts for the circular nature
	 * of hue values when computing distances.
	 *
	 * @return Detected color: red, blue, or none if uncertain
	 */
	color get_block_color(void);
	
	/**
	 * @brief Calculates the angular distance between two hue values
	 *
	 * Computes the shortest angular distance between two hue values on a
	 * circular scale (0-360 degrees). This accounts for the wrap-around
	 * nature of hue values, where 0 and 360 are adjacent.
	 *
	 * @param[in] a First hue value in degrees
	 * @param[in] b Second hue value in degrees
	 *
	 * @return Angular distance in degrees, in range [0, 180]
	 */
	float angularDistance(float a, float b) {
    	float d = fabs(a - b);
    	if (d > 180.0) d = 360.0 - d;
    	return d;
	}
};

#endif // INTAKE_HPP
