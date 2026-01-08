#ifndef SYSTEM_IDENTIFICATION_HPP
#define SYSTEM_IDENTIFICATION_HPP

/**
 * \file system-identification.hpp
 * @brief Motor system identification routines.
 *
 * Declares the `SysIdent` class, which performs system identification to
 * determine feedforward constants (K_v, K_a, K_s) for DC motors. The
 * routines apply test voltages, measure steady-state and transient
 * responses, and fit model parameters for use in feedforward control.
 */

#include "api.h"
#include "structs.hpp"
#include "helper-functions.hpp"
#include "motor-controller.hpp"

class MotorController;

/**
 * @brief System identification for motor characterization
 *
 * Performs system identification to determine feedforward constants (K_v, K_a, K_s)
 * for DC motors. The identification process involves applying known voltages,
 * measuring steady-state velocities and accelerations, and fitting the data to
 * the motor model. This is typically done once per motor to calibrate control.
 */
class SysIdent {
private:
	/**
	 * @brief Calculates mean steady-state velocities for motors at a given voltage
	 *
	 * Applies a specified voltage to all motors, waits for them to reach steady state,
	 * then samples velocities over multiple iterations to compute mean values.
	 * This data is used for identifying velocity-related constants (K_v, K_s).
	 *
	 * @param[in] voltage Voltage to apply to all motors in volts
	 * @param[in,out] motors Vector of motor controllers to test
	 *
	 * @return Vector of mean velocities in rad/s, one per motor
	 */
	static std::vector<float> calculate_mean_velocities(float voltage, std::vector<MotorController>& motors);

	/**
	 * @brief Calculates K_v and K_s constants for all motors
	 *
	 * Performs steady-state testing at multiple voltages to determine the velocity
	 * constant (K_v) and static friction constant (K_s) for each motor. The method
	 * applies various voltages, measures steady-state velocities, and fits the
	 * model V = K_v * omega + K_s * sign(omega) to the data.
	 *
	 * @param[in,out] motors Vector of motor controllers to characterize
	 *
	 * @return Vector of (K_v, K_s) pairs, one per motor
	 */
	static std::vector<std::pair<float,float>> calculate_Kv_and_Ks_s(std::vector<MotorController>& motors);

	/**
	 * @brief Fits K_v and K_s constants from voltage-velocity data
	 *
	 * Performs least-squares fitting to determine K_v and K_s from a set of
	 * (velocity, voltage) data points. The model accounts for the sign of velocity
	 * to properly handle static friction in both directions.
	 *
	 * @param[in] points Vector of (velocity, voltage) data points
	 *
	 * @return Pair of (K_v, K_s) constants
	 */
	static std::pair<float,float> fit_Kv_and_Ks_with_sign(const std::vector<std::pair<float,float>>& points);
	
	/**
	 * @brief Performs linear regression on a set of points
	 *
	 * Fits a linear model y = m*x + b to a set of (x, y) data points using
	 * least-squares regression. Returns the slope and intercept of the best-fit line.
	 *
	 * @param[in] points Vector of (x, y) data points
	 *
	 * @return Pair of (slope, intercept) for the fitted line
	 */
	static std::pair<float,float> linear_reg(const std::vector<std::pair<float,float>>& points);

	/**
	 * @brief Collects acceleration data for motors at a given voltage
	 *
	 * Applies a voltage to motors and collects time-series data of voltage,
	 * velocity, and acceleration measurements. Uses filtering and derivative
	 * estimation to compute accelerations from velocity measurements. This
	 * data is used for identifying the acceleration constant (K_a).
	 *
	 * @param[in] voltage Voltage to apply to all motors in volts
	 * @param[in,out] motors Vector of motor controllers to test
	 *
	 * @return Tuple of (V, w, alpha) vectors, where each vector contains
	 *         measurements for all motors
	 */
	static std::tuple<std::vector<std::vector<float>>, std::vector<std::vector<float>>, std::vector<std::vector<float>>> get_acceleration_data(float voltage, std::vector<MotorController>& motors);

	/**
	 * @brief Calculates K_a constants for all motors
	 *
	 * Uses acceleration data collected at a known voltage to determine the
	 * acceleration constant (K_a) for each motor. The method fits the model
	 * accounting for known K_v and K_s values to isolate K_a.
	 *
	 * @param[in] Kv_and_Ks_s Previously determined K_v and K_s values for each motor
	 * @param[in,out] motors Vector of motor controllers to characterize
	 *
	 * @return Vector of K_a values, one per motor
	 */
	static std::vector<float> calculate_Ka_s(std::vector<std::pair<float, float>> Kv_and_Ks_s, std::vector<MotorController>& motors);

	/**
	 * @brief Fits K_a constant using data that must pass through origin
	 *
	 * Performs a linear fit with the constraint that the line passes through
	 * the origin. This is appropriate for fitting K_a because acceleration
	 * should be zero when the voltage term (V - K_v*w - K_s*s) is zero.
	 * Filters out low-acceleration and low-velocity data points for robustness.
	 *
	 * @param[in] V Vector of voltage measurements
	 * @param[in] w Vector of velocity measurements in rad/s
	 * @param[in] alpha Vector of acceleration measurements in rad/s^2
	 * @param[in] Kv Previously determined velocity constant
	 * @param[in] Ks Previously determined static friction constant
	 *
	 * @return Fitted K_a constant
	 */
	static float through_origin_fit(const std::vector<float>& V, const std::vector<float>& w, const std::vector<float>& alpha, float Kv, float Ks);

public:
	/**
	 * @brief Calculates all feedforward constants for a set of motors
	 *
	 * Performs complete system identification to determine K_v, K_a, and K_s
	 * for all motors. The process involves steady-state testing for K_v and K_s,
	 * followed by acceleration testing for K_a. This is the main entry point
	 * for motor characterization.
	 *
	 * @param[in,out] motors Vector of motor controllers to characterize
	 *
	 * @return Vector of (K_v, K_a, K_s) tuples, one per motor
	 */
	static std::vector<std::tuple<float, float, float>> calculate_Kv_Ka_and_Ks_s(std::vector<MotorController>& motors);
	
	/**
	 * @brief Calculates and prints feedforward constants for motors
	 *
	 * Performs system identification on all motors and prints the results
	 * to the console in a formatted manner. Each motor's constants are
	 * printed with its name identifier. This function blocks during the
	 * identification process.
	 *
	 * @param[in,out] motors Vector of motor controllers to characterize and print
	 */
	static void calculate_and_print_constants(std::vector<MotorController>& motors);
};

#endif // SYSTEM_IDENTIFICATION_HPP
