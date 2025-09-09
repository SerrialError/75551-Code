#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"
#include "structs.hpp"

class drivetrain {
private:
    pros::Motor& m1;
    pros::Motor& m2;
    pros::Motor& o1;
    pros::Motor& o2;
    pros::Motor& m3;
    pros::Motor& m4;
    double wheelbase_length;
    double trackwidth_length;
public:
    drivetrain(pros::Motor& m1_, pros::Motor& m2_, pros::Motor& o1_, pros::Motor& o2_, pros::Motor& m3_, pros::Motor& m4_, double wheelbase_length_, double trackwidth_length_)
        : m1(m1_), m2(m2_), o1(o1_), o2(o2_), m3(m3_), m4(m4_), wheelbase_length(wheelbase_length_), trackwidth_length(trackwidth_length_) {}

    wheel_vels calculate_wheel_vels(pose_vels desired_vels, wheel_vel_lims wheel_vel_limits);
};

#endif // DRIVETRAIN_HPP
