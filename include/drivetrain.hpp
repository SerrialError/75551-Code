#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"
#include "structs.hpp"
#include "ff-velocity-controller.hpp"
#include "localization.hpp"
#include "helper-functions.hpp"
#include "motor-controller.hpp"
#include "motion-profiler.hpp"
#include "system-identification.hpp"

class drivetrain {
private:
	wheels<MotorController> motors;
	const double wheelbase_length;
	const double trackwidth_length;
	static constexpr double wheel_radius = 2.0 / 2.0 * 0.0254;
	static constexpr double b_gain = 2.0;
	static constexpr double decimal_of_max_velocity = 0.195;
	static constexpr double decimal_of_max_acceleration = 0.3;
	static constexpr double gear_ratio = 48.0/36.0;
	const double max_wheels_ang_vel;
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
			pose Pose_);
	const double max_robot_lin_vel_scaled;
	const double max_robot_ang_vel;
	const double max_robot_ang_vel_scaled;
	const double max_robot_ang_accel_scaled;
	Localization localization;
	mp LinearMP;
	mp AngularMP;
	double angular_velocity(const double angular_acceleration, ff_constants motor_constant) {
		return ((motor_constant.max_voltage-(motor_constant.K_a * angular_acceleration)-motor_constant.K_s)/motor_constant.K_v);
	};

	double angular_acceleration(const double angular_velocity, ff_constants motor_constant) {
		return ((motor_constant.max_voltage-(motor_constant.K_v * angular_velocity)-motor_constant.K_s)/motor_constant.K_a);
	};

	pros::Controller master{pros::E_CONTROLLER_MASTER};

	void motor_brakes();

	wheels<wheel_vel_bounds> calculate_wheel_vels_bounds(const pose& desired_vels, const wheels<wheel_vel_bounds>& bounds);

	std::optional<wheels<double>> calculate_wheel_vels(const pose& desired_vels, const wheels<wheel_vel_bounds>& bounds);

	void move_motor_volts(const wheels<double>& wheel_voltages);

	void move_motor_volts_time(const wheels<double>& wheel_voltages, const int time);

	void field_oriented_holonomic_control(const double dt);

	double get_standard_angle(void);

	double joystick_to_vel(const double joystick);

	wheels<wheel_vel_bounds> get_wheel_vel_bounds(const double dt);

	wheels<motorVelocityType> get_wanted_motor_accels(const wheels<motorVelocityType>& desired_motor_vels, const double dt);

	void tank_drive_control(const double dt);

	wheels<motorVelocityType> differential_vels_to_motor_vels(differentialVels robot_velocity);	

	void move_differential_robot_vels(const std::vector<differentialVels>& robot_vels, const double dt);

	differentialVels ramsete(pose wanted_pose, differentialVels wanted_vels);

	void move_differential_robot_vels_ramsete(std::vector<differentialVels> robot_vels, std::vector<pose>, const double dt);

	void move_motor_accelerations(const wheels<motorVelocityType>& motor_accelerations);

	wheels<double> bound_desired_motor_velocities(const wheels<double>& desired_motor_velocities, const double dt);

	void linear_mp(const double distance);
	
	void angular_mp(const double angle);
	
	void mtp_mp(const pose desired_pose);
		
	void mtp_mp_ramsete(const pose desired_pose);

	void calculate_and_print_motor_constants();
};

#endif // DRIVETRAIN_HPP
