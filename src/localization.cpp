#include "localization.hpp"
#include "helper-functions.hpp"

void localization::calculate_pose(void) {
    double delta_angle = DEG_TO_RAD_NORM(imu.get_rotation()) - prev_angle;
    double delta_linear_wheel = linear_wheel.get_angle() * 2.0 * 0.0254 - prev_linear_wheel;
    double delta_x = ((delta_linear_wheel) / (DEG_TO_RAD_NORM(imu.get_rotation()) - prev_angle)) * (cos((delta_angle)) + 1.0); 
    double delta_y = ((delta_linear_wheel) / (DEG_TO_RAD_NORM(imu.get_rotation()) - prev_angle)) * (sin((delta_angle)));
    prev_angle = DEG_TO_RAD_NORM(imu.get_rotation());
    prev_linear_wheel = linear_wheel.get_angle() * 2.0 * 0.0254;
    Pose = {Pose.x + delta_x, Pose.y + delta_y, Pose.theta + delta_angle};
}
