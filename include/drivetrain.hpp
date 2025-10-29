#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"
#include "structs.hpp"
#include "ff-velocity-controller.hpp"

class drivetrain {
private:
    const wheels<std::reference_wrapper<pros::Motor>> motors;
    pros::Imu& imu;
    const double wheelbase_length;
    const double trackwidth_length;
    const wheels<ff_constants> motor_constants;
    const wheels<DCff> motor_ffs;
    const double x_max_velocity;
    const double y_max_velocity;
    const pose initial_pose = {0, 0, 0};

public:
    drivetrain(const wheels<std::reference_wrapper<pros::Motor>>& motors_,
              pros::Imu& imu_,
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
                     DCff(motor_constants_.m4) },
          x_max_velocity((motor_constants.m1.max_ang_vel + motor_constants.m2.max_ang_vel + motor_constants.o1.max_ang_vel + motor_constants.o2.max_ang_vel + motor_constants.m3.max_ang_vel + motor_constants.m4.max_ang_vel) / 6.f * 2.f * 0.0254f),
          y_max_velocity((motor_constants.m1.max_ang_vel + motor_constants.m2.max_ang_vel + motor_constants.o1.max_ang_vel + motor_constants.o2.max_ang_vel + motor_constants.m3.max_ang_vel + motor_constants.m4.max_ang_vel) / 6.f * 2.f * 0.0254f)
    {}

    pros::Controller master{pros::E_CONTROLLER_MASTER};
    double sgn(double x) {
        if (x > 0) {
            return 1.0;
        } else if (x < 0) {
            return -1.0;
        } else {
            return 0.0;
        }
    }
	wheels<wheel_vel_lim> calculate_wheel_vels_bounds(const pose& desired_vels, const wheels<wheel_vel_lim>& limits);
	wheels<double> calculate_wheel_vels(const pose& desired_vels, const wheels<wheel_vel_lim>& limits);


	wheels<double> calculate_wheel_vels(const wheels<wheel_vel_lim>& bounds);

	double get_motor_max_accel(pros::Motor motor, const ff_constants motor_constants_);
    
    void move_wheel_accels(const wheels<double>& wheel_accelerations);

    void move_wheel_volts(const wheels<double>& wheel_voltages);
    
    void field_oriented_holonomic_control(const double& dt);
    
    double get_standard_angle(void);
    
    double joystick_to_vel(const double& joystick);
    
    wheels<wheel_vel_lim> get_motor_vel_limits(const double& dt);
    
    wheels<double> get_wanted_motor_accels(const wheels<double>& wanted_motor_vels, const double& dt);

    void tank_drive_control();
	
	void test_control(const double& dt);
	
	double get_wanted_motor_vel(pros::Motor& motor, const ff_constants motor_constants_, const double wanted_velocity, const double& dt);
};

#endif // DRIVETRAIN_HPP
