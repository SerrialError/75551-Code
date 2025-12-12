#include "drivetrain.hpp"
#include "helper-functions.hpp"
#include "structs.hpp"

drivetrain::drivetrain(const wheels<std::reference_wrapper<pros::Motor>>& motors_,
                       const double wheelbase_length_,
                       const double trackwidth_length_,
                       const wheels<ff_constants>& motor_constants_,
                       pros::Rotation& linear_wheel_,
                       pros::Rotation& horizontal_wheel_,
                       pros::Imu& imu_,
                       pose Pose_)
    : motors{
          MotorController(motors_.m1, motor_constants_.m1, "m1"),
          MotorController(motors_.m2, motor_constants_.m2, "m2"),
          MotorController(motors_.o1, motor_constants_.o1, "o1"),
          MotorController(motors_.o2, motor_constants_.o2, "o2"),
          MotorController(motors_.m3, motor_constants_.m3, "m3"),
          MotorController(motors_.m4, motor_constants_.m4, "m4")},
      wheelbase_length(wheelbase_length_),
      trackwidth_length(trackwidth_length_),
      localization{linear_wheel_, horizontal_wheel_, imu_, Pose_},
      max_wheels_ang_vel(
          std::min({ angular_velocity(0.0, motor_constants_.m1),
                     angular_velocity(0.0, motor_constants_.m2),
                     angular_velocity(0.0, motor_constants_.o1),
                     angular_velocity(0.0, motor_constants_.o2),
                     angular_velocity(0.0, motor_constants_.m3),
                     angular_velocity(0.0, motor_constants_.m4) }) * gear_ratio),
      max_wheels_ang_vel_scaled(max_wheels_ang_vel * decimal_of_max_velocity),
      min_wheels_ang_accel(
          std::min({ angular_acceleration(max_wheels_ang_vel_scaled, motor_constants_.m1),
                     angular_acceleration(max_wheels_ang_vel_scaled, motor_constants_.m2),
                     angular_acceleration(max_wheels_ang_vel_scaled, motor_constants_.o1),
                     angular_acceleration(max_wheels_ang_vel_scaled, motor_constants_.o2),
                     angular_acceleration(max_wheels_ang_vel_scaled, motor_constants_.m3),
                     angular_acceleration(max_wheels_ang_vel_scaled, motor_constants_.m4) })),
      max_robot_lin_vel_scaled(max_wheels_ang_vel_scaled * wheel_radius),
      max_robot_ang_vel(((wheelbase_length_ + trackwidth_length_) / 24.0 * (max_wheels_ang_vel * wheel_radius * 4.0)) +
                        (trackwidth_length_ / 12.0 * (max_wheels_ang_vel * wheel_radius * 2.0))),
      max_robot_ang_vel_scaled(((wheelbase_length_ + trackwidth_length_) / 24.0 * (max_wheels_ang_vel_scaled * wheel_radius * 4.0)) +
                               (trackwidth_length_ / 12.0 * (max_wheels_ang_vel_scaled * wheel_radius * 2.0))),
      max_robot_ang_accel_scaled(((wheelbase_length_ + trackwidth_length_) / 24.0 * (min_wheels_ang_accel * decimal_of_max_acceleration * wheel_radius * 4.0)) +
                                 (trackwidth_length_ / 12.0 * (min_wheels_ang_accel * decimal_of_max_acceleration * wheel_radius * 2.0))),
      LinearMP((min_wheels_ang_accel * decimal_of_max_acceleration * wheel_radius), (max_robot_lin_vel_scaled)),
      AngularMP((max_robot_ang_accel_scaled), (max_robot_ang_vel_scaled))
{}

void drivetrain::motor_brakes() {
	for (size_t i = 0; i < 6; ++i) {
		motors[i].motor.get().set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
		motors[i].motor.get().brake();
		move_motor_volts({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	}
}

void drivetrain::move_motor_volts(const wheels<double>& wheel_voltages) {
    for (size_t i = 0; i < 6; ++i) {
        motors[i].move_motor_voltage(wheel_voltages[i]);
    }
}

void drivetrain::move_motor_accelerations(const wheels<motorVelocityType>& motor_accelerations) {
    for (int i = 0; i < 6; i++) {
	    motors[i].move_motor_acceleration(motor_accelerations[i]);
    }
}

void drivetrain::move_motor_volts_time(const wheels<double>& wheel_voltages, const int time) {
    for (size_t i = 0; i < time / 10; i++) {
        move_motor_volts(wheel_voltages);
	    pros::delay(10);
    }
}

wheels<wheel_vel_bounds> drivetrain::get_wheel_vel_bounds(const double dt) {
    wheels<wheel_vel_bounds> result{};
    for (size_t i = 0; i < 6; ++i) {
	    result[i].min = motors[i].get_motor_vel_bounds(dt).min * wheel_radius;
	    result[i].max = motors[i].get_motor_vel_bounds(dt).max * wheel_radius;
    }
    return result;
}

wheels<motorVelocityType> drivetrain::get_wanted_motor_accels(const wheels<motorVelocityType>& desired_motor_vels, const double dt) {
    wheels<motorVelocityType> result{};
    for (size_t i = 0; i < 6; ++i) {
	    result[i].velocity = motors[i].get_desired_motor_acceleration(desired_motor_vels[i].velocity, dt);
		result[i].brakeMode = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
    }
    return result;
}

void drivetrain::tank_drive_control(const double dt) {
    const double x_right = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    const double y_left = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    // for (int j = 0; j < 6; ++j) {
	    // motors[j].update_motor_data();
    // }
    const wheels<wheel_vel_bounds> bounds = get_wheel_vel_bounds(dt);
    const double W = trackwidth_length;
	const double L = wheelbase_length;
    double linear_velocity;
    double angular_velocity;
    if (y_left > 0) {
         const double lin_max_velocity = (bounds.m1.max + bounds.m2.max + bounds.m3.max + bounds.m4.max + bounds.o1.max + bounds.o2.max) / 6.0;
	 linear_velocity = std::abs(y_left) / 127.0 * lin_max_velocity;
    }
    else {
         const double lin_min_velocity = (bounds.m1.min + bounds.m2.min + bounds.m3.min + bounds.m4.min + bounds.o1.min + bounds.o2.min) / 6.0;
	 linear_velocity = std::abs(y_left) / 127.0 * lin_min_velocity;
    }
    if (-x_right > 0) {
    	const double ang_max_velocity = (L+W)/24.0*(bounds.m2.max + bounds.m4.max - bounds.m1.min - bounds.m3.min) + W/12.0*(bounds.o2.max-bounds.o1.min);
	angular_velocity = std::abs(x_right) / 127.0 * ang_max_velocity;
    }
    else {
    	const double ang_min_velocity = (L+W)/24.0*(bounds.m2.min + bounds.m4.min - bounds.m1.max - bounds.m3.max) + W/12.0*(bounds.o2.min-bounds.o1.max);
	angular_velocity = std::abs(x_right) / 127.0 * ang_min_velocity;

    }
	const differentialVels robot_velocity = {linear_velocity, angular_velocity};
	const wheels<motorVelocityType> wanted_motor_vels = differential_vels_to_motor_vels(robot_velocity);
    // wheels<double> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
    const wheels<motorVelocityType> wanted_motor_accels = get_wanted_motor_accels(wanted_motor_vels, dt);
    move_motor_accelerations(wanted_motor_accels);
}

wheels<double> drivetrain::bound_desired_motor_velocities(const wheels<double>& desired_motor_velocities, const double dt) {
    wheels<double> result;
    for (size_t i = 0; i < 6; ++i) {
	    result[i] = motors[i].bound_desired_motor_velocity(desired_motor_velocities[i], dt);
    }
    return result;
}

wheels<motorVelocityType> drivetrain::differential_vels_to_motor_vels(differentialVels robot_velocity) {	
    const double L = wheelbase_length;
    const double W = trackwidth_length;
	const double m1_velocity = (3.0/4.0 * robot_velocity.linear - (3.0*robot_velocity.angular)/(L+W)) / wheel_radius;
	const double m2_velocity = (3.0/4.0 * robot_velocity.linear + (3.0*robot_velocity.angular)/(L+W)) / wheel_radius;
	const double m3_velocity = (3.0/4.0 * robot_velocity.linear - (3.0*robot_velocity.angular)/(L+W)) / wheel_radius;
	const double m4_velocity = (3.0/4.0 * robot_velocity.linear + (3.0*robot_velocity.angular)/(L+W)) / wheel_radius;
	const double o1_velocity = (1.50*robot_velocity.linear-3.0*robot_velocity.angular/(W)) / wheel_radius;
	const double o2_velocity = (1.50*robot_velocity.linear+3.0*robot_velocity.angular/(W)) / wheel_radius;
	return {{m1_velocity, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {m2_velocity, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {o1_velocity, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {o2_velocity, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {m3_velocity, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {m4_velocity, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}};
}

void drivetrain::move_differential_robot_vels(const std::vector<differentialVels>& robot_vels, const double dt) {
	for (size_t i = 0; i < robot_vels.size(); i++) {
		wheels<motorVelocityType> wanted_motor_vels = differential_vels_to_motor_vels(robot_vels[i]);
    	// wheels<double> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
    	const wheels<motorVelocityType> wanted_motor_accels = get_wanted_motor_accels(wanted_motor_vels, dt);
		move_motor_accelerations(wanted_motor_accels);
	}
}

differentialVels drivetrain::ramsete(pose wanted_pose, differentialVels wanted_vels) {
	 // Compute errors in robot frame
    double error_theta = wrapToPi(wanted_pose.theta - localization.Pose.theta);
    double dx = wanted_pose.x - localization.Pose.x;
    double dy = wanted_pose.y - localization.Pose.y;
    double cos_t = std::cos(localization.Pose.theta);
    double sin_t = std::sin(localization.Pose.theta);
    double error_x =  sin_t * dy + cos_t * dx;
    double error_y =  cos_t * dy - sin_t * dx;

    // Gains
    double k2 = std::sqrt(wanted_vels.angular * wanted_vels.angular + (b_gain * wanted_vels.linear) * (b_gain * wanted_vels.linear));

    // Compute control outputs
    double linear_out  = wanted_vels.linear * std::cos(error_theta) + b_gain * error_x;
    double angular_out = wanted_vels.angular + k2 * sinc(error_theta) * error_y + b_gain * error_theta;
	return {linear_out, angular_out};
}

void drivetrain::move_differential_robot_vels_ramsete(std::vector<differentialVels> robot_vels, std::vector<pose> wanted_pose, const double dt) {
	for (size_t i = 0; i < robot_vels.size(); i++) {
		differentialVels corrected_robot_vels = ramsete(wanted_pose[i], robot_vels[i]); 
		wheels<motorVelocityType> wanted_motor_vels = differential_vels_to_motor_vels(corrected_robot_vels);
    	// wheels<double> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
    	const wheels<motorVelocityType> wanted_motor_accels = get_wanted_motor_accels(wanted_motor_vels, dt);
		move_motor_accelerations(wanted_motor_accels);
        pros::delay(static_cast<int>(dt*1000.0));
	}
}

void drivetrain::linear_mp(const double distance) {
    double time = 0.0;
    bool start = true;
    while(!LinearMP.profileFinished(time) || start) {
        double linear_velocity = LinearMP.velocity(time, distance);
	    std::vector<differentialVels> desired_differential_vel = {{linear_velocity * 1.0, 0.0}};
	    move_differential_robot_vels(desired_differential_vel, .01);
        pros::delay(10);
        time += 0.01;
        start = false;
    }
    motor_brakes();
}

void drivetrain::angular_mp(const double angle) {
    double time = 0.0;
    bool start = true;
    while(!AngularMP.profileFinished(time) || start) {
        double angular_velocity = AngularMP.velocity(time, angle);
	    std::vector<differentialVels> desired_differential_vel = {{0.0, angular_velocity}};
	    move_differential_robot_vels(desired_differential_vel, .01);
        pros::delay(10);
        time += 0.01;
        start = false;
    }
    motor_brakes();
}

void drivetrain::calculate_and_print_motor_constants() {
	std::vector<MotorController> motors_;
	motors_.reserve(6);  
	for (size_t i = 0; i < 6; ++i) {
		motors_.push_back(motors[i]);
	}
	SysIdent::calculate_and_print_constants(motors_);
}

void drivetrain::mtp_mp(const pose desired_pose) {
	const double desired_angle_delta = localization.Pose.theta - desired_pose.theta;
	angular_mp(desired_angle_delta);
	pros::delay(500);
	const double desired_linear_distance_delta = std::hypot(localization.Pose.x - desired_pose.x, localization.Pose.y - desired_pose.y);
	linear_mp(desired_angle_delta);
};
	
void drivetrain::mtp_mp_ramsete(const pose desired_pose) {

}
