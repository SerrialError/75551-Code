#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

/**
 * \file localization.hpp
 * @brief Odometry-based robot localization.
 *
 * Declares the `Localization` class that estimates the robot's pose using
 * rotation sensors and an IMU. This file provides the interface for pose
 * tracking and manual pose resetting used throughout the drivetrain code.
 */

#include "api.h"
#include "structs.hpp"
#include "helper-functions.hpp"

/**
 * @brief Robot pose estimation using odometry sensors
 *
 * Tracks the robot's position and orientation on the field using a combination
 * of rotation sensors for linear and horizontal movement and an IMU for heading.
 * The class maintains the current pose and provides methods to update it based
 * on sensor readings.
 */
class Localization {
private:
    pros::Rotation& linear_wheel;
    pros::Rotation& horizontal_wheel;
    pros::Imu& imu;
    float prev_angle = 0.0;
    float prev_linear_wheel = 0.0;
    float prev_horizontal_wheel = 0.0;

public:
    /**
     * @brief Constructs a localization system with sensors and initial pose
     *
     * Initializes the localization system with references to rotation sensors
     * for tracking linear and horizontal movement, an IMU for orientation, and
     * an initial pose. The system will track changes from this initial position.
     *
     * @param[in] linear_wheel_ Rotation sensor for forward/backward movement tracking
     * @param[in] horizontal_wheel_ Rotation sensor for sideways movement tracking
     * @param[in] imu_ Inertial measurement unit for heading/orientation tracking
     * @param[in] Pose_ Initial pose (x, y, theta) of the robot on the field
     */
    Localization(pros::Rotation& linear_wheel_,
              pros::Rotation& horizontal_wheel_,
              pros::Imu& imu_,
              pose Pose_)
        : linear_wheel(linear_wheel_),
          horizontal_wheel(horizontal_wheel_),
          imu(imu_),
          Pose{Pose_}
    {}

    pose Pose;
    
	/**
	 * @brief Calculates and updates the robot's current pose
	 *
	 * Updates the robot's pose by integrating sensor readings. Computes the
	 * change in angle from the IMU and the change in linear wheel position,
	 * then uses these to calculate the change in x and y coordinates. The
	 * calculation accounts for the robot's rotation when computing position
	 * changes. This function should be called periodically to maintain
	 * accurate pose estimation.
	 */
	void calculate_pose(void);

	/**
	 * @brief Sets the robot's pose to a specified value
	 *
	 * Manually sets the robot's pose, typically used for resetting position
	 * after a known reference point or for initialization. This is useful
	 * for field reset or when using vision-based pose correction.
	 *
	 * @param[in] inputed_pose New pose (x, y, theta) to set
	 */
	void set_pose(pose inputed_pose) {
		Pose = inputed_pose;
	}
};

#endif // LOCALIZATION_HPP
