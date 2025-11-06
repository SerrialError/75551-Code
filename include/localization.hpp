#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include "api.h"
#include "structs.hpp"
#include "ff-velocity-controller.hpp"
#include "helper-functions.hpp"

class Localization {
private:
    pros::Rotation& linear_wheel;
    pros::Rotation& horizontal_wheel;
    pros::Imu& imu;
    double prev_angle = 0.0;
    double prev_linear_wheel = 0.0;
    double prev_horizontal_wheel = 0.0;

public:
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
    
	void calculate_pose(void);
};

#endif // LOCALIZATION_HPP
