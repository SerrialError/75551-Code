#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"
#include "structs.hpp"
#include "ff-velocity-controller.hpp"

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
    ff_constants m1_constants;
    ff_constants m2_constants;
    ff_constants o1_constants;
    ff_constants o2_constants;
    ff_constants m3_constants;
    ff_constants m4_constants;

public:
    drivetrain(pros::Motor& m1_, pros::Motor& m2_, pros::Motor& o1_, pros::Motor& o2_, pros::Motor& m3_, pros::Motor& m4_, double wheelbase_length_, double trackwidth_length_, 
    ff_constants m1_constants_, ff_constants m2_constants_, ff_constants o1_constants_, ff_constants o2_constants_, ff_constants m3_constants_, ff_constants m4_constants_)
        : m1(m1_), m2(m2_), o1(o1_), o2(o2_), m3(m3_), m4(m4_), wheelbase_length(wheelbase_length_), trackwidth_length(trackwidth_length_), 
        m1_constants(m1_constants_), m2_constants(m2_constants_), o1_constants(o1_constants_), o2_constants(o2_constants_), m3_constants(m3_constants_), m4_constants(m4_constants_)  {}

    wheel_vels calculate_wheel_vels(pose_vels desired_vels, wheel_vel_lims wheel_vel_limits);
    void move_wheel_vels(wheel_vels wheel_velocities);
};

#endif // DRIVETRAIN_HPP
