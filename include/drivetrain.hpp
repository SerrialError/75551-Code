#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"
#include "structs.hpp"
#include "ff-velocity-controller.hpp"

class drivetrain {
private:
    const pros::Controller master{pros::E_CONTROLLER_MASTER};
    const wheels<std::reference_wrapper<pros::Motor>> motors;
    const pros::Imu& imu;
    const double wheelbase_length;
    const double trackwidth_length;
    const wheels<ff_constants> motor_constants;
    const wheels<DCff> motor_ffs;

public:
    drivetrain(const wheels<std::reference_wrapper<pros::Motor>>& motors_,
              const pros::Imu& imu_,
              const double wheelbase_length_,
              const double trackwidth_length_,
              const wheels<ff_constants>& motor_constants_)
        : motors(motors_),
          imu(imu_),
          wheelbase_length(wheelbase_length_),
          trackwidth_length(trackwidth_length_),
          motor_constants(motor_constants_),
          motor_ffs{ DCff(motor_constants_.m1),
                     DCff(motor_constants_.m2),
                     DCff(motor_constants_.o1),
                     DCff(motor_constants_.o2),
                     DCff(motor_constants_.m3),
                     DCff(motor_constants_.m4) }
    {}

    wheels<double> calculate_wheel_vels(const pose& desired_vels, const wheels<wheel_vel_lim>& wheel_vel_limits);
    
    void move_wheel_accels(const wheels<double>& wheel_accelerations);
    
    void field_oriented_holonomic_control();
};

#endif // DRIVETRAIN_HPP
