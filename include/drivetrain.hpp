#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"
#include "structs.hpp"
#include "ff-velocity-controller.hpp"
#include "localization.hpp"
#include "helper-functions.hpp"

class drivetrain {
private:
    const wheels<std::reference_wrapper<pros::Motor>> motors;
    const double wheelbase_length;
    const double trackwidth_length;
    const wheels<ff_constants> motor_constants;
    const wheels<DCff> motor_ffs;
	Localization localization;
	const double wheel_radius = 2.0 / 2.0 * 0.0254;
	const double b_gain = 2.0;
    const double decimal_of_max_velocity = 0.7;
    const double max_wheels_ang_vel_scaled;
    const double min_wheels_ang_accel;

public:
    drivetrain(const wheels<std::reference_wrapper<pros::Motor>>& motors_,
              const double wheelbase_length_,
              const double trackwidth_length_,
              const wheels<ff_constants>& motor_constants_,
			  pros::Rotation& linear_wheel_,
              pros::Rotation& horizontal_wheel_,
              pros::Imu& imu_,
              pose Pose_)
        : motors(motors_),
          wheelbase_length(wheelbase_length_),
          trackwidth_length(trackwidth_length_),
          motor_constants(motor_constants_),
          motor_ffs{ DCff(motor_constants_.m1),
                     DCff(motor_constants_.m2),
                     DCff(motor_constants_.o1),
                     DCff(motor_constants_.o2),
                     DCff(motor_constants_.m3),
                     DCff(motor_constants_.m4) },
		  localization{linear_wheel_, horizontal_wheel_, imu_, Pose_},
          max_wheels_ang_vel_scaled(std::min({angular_velocity(0.0, motor_constants_.m1) * decimal_of_max_velocity, angular_velocity(0.0, motor_constants_.m2) * decimal_of_max_velocity, angular_velocity(0.0, motor_constants_.o1) * decimal_of_max_velocity, angular_velocity(0.0, motor_constants_.o2) * decimal_of_max_velocity, angular_velocity(0.0, motor_constants_.m3) * decimal_of_max_velocity, angular_velocity(0.0, motor_constants_.m4) * decimal_of_max_velocity})),
          min_wheels_ang_accel(std::min({angular_acceleration(max_wheels_ang_vel_scaled, motor_constants_.m1), angular_acceleration(max_wheels_ang_vel_scaled, motor_constants_.m2), angular_acceleration(max_wheels_ang_vel_scaled, motor_constants_.o1), angular_acceleration(max_wheels_ang_vel_scaled, motor_constants_.o2), angular_acceleration(max_wheels_ang_vel_scaled, motor_constants_.m3), angular_acceleration(max_wheels_ang_vel_scaled, motor_constants_.m4)}))
    {}
    
    double angular_velocity(const double angular_acceleration, ff_constants motor_constant) {
        return ((motor_constant.max_voltage-(motor_constant.K_a * angular_acceleration)-motor_constant.K_s)/motor_constant.K_v);
    };

    double angular_acceleration(const double angular_velocity, ff_constants motor_constant) {
        return ((motor_constant.max_voltage-(motor_constant.K_v * angular_velocity)-motor_constant.K_s)/motor_constant.K_a);
    };

    pros::Controller master{pros::E_CONTROLLER_MASTER};
	
	wheels<wheel_vel_bounds> calculate_wheel_vels_bounds(const pose& desired_vels, const wheels<wheel_vel_bounds>& bounds);

    std::optional<wheels<double>> calculate_wheel_vels(const pose& desired_vels, const wheels<wheel_vel_bounds>& bounds);

	double get_motor_max_accel(pros::Motor& motor, const ff_constants motor_constants_, int direction);
    
    void move_wheel_accels(const wheels<double>& wheel_accelerations);

    void move_wheel_volts(const wheels<double>& wheel_voltages);

    void move_wheel_volts_time(const wheels<double>& wheel_voltages, const int time);
    
    void field_oriented_holonomic_control(const double& dt);
    
    double get_standard_angle(void);
    
    double joystick_to_vel(const double& joystick);
    
    wheels<wheel_vel_bounds> get_wheel_vel_bounds(const double& dt);
    
    wheels<double> get_wanted_motor_accels(const wheels<double>& wanted_motor_vels, const double& dt);

    void tank_drive_control(const double& dt);
	
	double get_wanted_motor_vel(pros::Motor& motor, const ff_constants motor_constants_, const double wanted_velocity, const double& dt);


	wheels<double> differential_vels_to_motor_vels(differentialVels robot_velocity);	

	void move_differential_robot_vels(std::vector<differentialVels> robot_vels, const double& dt);

	differentialVels ramsete(pose wanted_pose, differentialVels wanted_vels);

	void move_differential_robot_vels_ramsete(std::vector<differentialVels> robot_vels, std::vector<pose>, const double& dt);
};

#endif // DRIVETRAIN_HPP
