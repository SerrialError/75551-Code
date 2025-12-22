/**
 * \file localization.cpp
 * @brief Implementation of odometry-based pose updates.
 *
 * Integrates readings from the IMU and tracking wheels to update the
 * robot's pose in the field frame. The algorithm assumes small time
 * steps between updates and uses the change in heading to compute the
 * arc traveled by the tracking wheel.
 */
#include "localization.hpp"
#include "helper-functions.hpp"

void Localization::calculate_pose(void) {
    double delta_angle = DEG_TO_RAD_NORM(imu.get_rotation()) - prev_angle;
    double delta_linear_wheel = linear_wheel.get_angle() * 2.0 * 0.0254 - prev_linear_wheel;
    // Approximate the path as an arc: delta_linear_wheel is chord length, use
    // change in heading to recover local frame x/y deltas.
    double delta_x = ((delta_linear_wheel) / (DEG_TO_RAD_NORM(imu.get_rotation()) - prev_angle)) * (cos((delta_angle)) + 1.0); 
    double delta_y = ((delta_linear_wheel) / (DEG_TO_RAD_NORM(imu.get_rotation()) - prev_angle)) * (sin((delta_angle)));
    prev_angle = DEG_TO_RAD_NORM(imu.get_rotation());
    prev_linear_wheel = linear_wheel.get_angle() * 2.0 * 0.0254;
    Pose = {Pose.x + delta_x, Pose.y + delta_y, Pose.theta + delta_angle};
}
