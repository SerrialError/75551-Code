/**
 * \file drivetrain.cpp
 * @brief Implementation of drivetrain control, RAMSETE, and motion profiles.
 *
 * Contains the implementation of the holonomic drivetrain, including tank
 * drive, field-oriented control, RAMSETE-based path following, and motion
 * profiling helpers for linear and angular movements.
 */
#include "drivetrain.hpp"
#include "helper-functions.hpp"
#include "structs.hpp"
#include "logger.hpp"

drivetrain::drivetrain(const wheels<std::reference_wrapper<pros::Motor>>& motors_,
                       const float wheelbase_length_,
                       const float trackwidth_length_,
                       const float timestep_,
                       const wheels<FirstOrderFeedforwardConstants>& ff_motor_constants_,
                       const wheels<PIDConstants>& pid_motor_constants_,
                       pros::Rotation& linear_wheel_,
                       pros::Rotation& horizontal_wheel_,
                       pros::Imu& imu_,
                       pose Pose_)
    : motors{
          MotorController(motors_.m1, ff_motor_constants_.m1, pid_motor_constants_.m1, timestep_, "m1"),
          MotorController(motors_.m2, ff_motor_constants_.m2, pid_motor_constants_.m2, timestep_, "m2"),
          MotorController(motors_.o1, ff_motor_constants_.o1, pid_motor_constants_.o1, timestep_, "o1"),
          MotorController(motors_.o2, ff_motor_constants_.o2, pid_motor_constants_.o2, timestep_, "o2"),
          MotorController(motors_.m3, ff_motor_constants_.m3, pid_motor_constants_.m3, timestep_, "m3"),
          MotorController(motors_.m4, ff_motor_constants_.m4, pid_motor_constants_.m4, timestep_, "m4")},
      wheelbase_length(wheelbase_length_),
      trackwidth_length(trackwidth_length_),
      localization{linear_wheel_, horizontal_wheel_, imu_, Pose_},
      max_wheels_ang_vel(
          std::min({ angular_velocity(0.f, ff_motor_constants_.m1),
                     angular_velocity(0.f, ff_motor_constants_.m2),
                     angular_velocity(0.f, ff_motor_constants_.o1),
                     angular_velocity(0.f, ff_motor_constants_.o2),
                     angular_velocity(0.f, ff_motor_constants_.m3),
                     angular_velocity(0.f, ff_motor_constants_.m4) }) * gear_ratio),
      max_wheels_ang_vel_scaled(max_wheels_ang_vel * decimal_of_max_velocity),
      min_wheels_ang_accel(
          std::min({ angular_acceleration(max_wheels_ang_vel_scaled, ff_motor_constants_.m1),
                     angular_acceleration(max_wheels_ang_vel_scaled, ff_motor_constants_.m2),
                     angular_acceleration(max_wheels_ang_vel_scaled, ff_motor_constants_.o1),
                     angular_acceleration(max_wheels_ang_vel_scaled, ff_motor_constants_.o2),
                     angular_acceleration(max_wheels_ang_vel_scaled, ff_motor_constants_.m3),
                     angular_acceleration(max_wheels_ang_vel_scaled, ff_motor_constants_.m4) })),
      // Maximum linear and angular speeds/accelerations that respect motor limits.
      max_robot_lin_vel_scaled(max_wheels_ang_vel_scaled * wheel_radius),
      max_robot_ang_vel(((wheelbase_length_ + trackwidth_length_) / 24.f * (max_wheels_ang_vel * wheel_radius * 4.f)) +
                        (trackwidth_length_ / 12.f * (max_wheels_ang_vel * wheel_radius * 2.f))),
      max_robot_ang_vel_scaled(((wheelbase_length_ + trackwidth_length_) / 24.f * (max_wheels_ang_vel_scaled * wheel_radius * 4.f)) +
                               (trackwidth_length_ / 12.f * (max_wheels_ang_vel_scaled * wheel_radius * 2.f))),
      max_robot_ang_accel_scaled(((wheelbase_length_ + trackwidth_length_) / 24.f * (min_wheels_ang_accel * decimal_of_max_acceleration * wheel_radius * 4.f)) +
                                 (trackwidth_length_ / 12.f * (min_wheels_ang_accel * decimal_of_max_acceleration * wheel_radius * 2.f))),
	  max_robot_lin_accel_scaled(min_wheels_ang_accel * decimal_of_max_acceleration * wheel_radius),
      timestep(timestep_)
{}

void drivetrain::motor_brakes() {
	for (size_t i = 0; i < 6; ++i) {
		motors[i].motor.get().set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
		motors[i].motor.get().brake();
		move_voltage({0.f, 0.f, 0.f, 0.f, 0.f, 0.f});
	}
}

void drivetrain::move_voltage(const wheels<float>& wheel_voltages) {
    for (size_t i = 0; i < 6; ++i) {
        motors[i].move_voltage(wheel_voltages[i]);
    }
}

void drivetrain::move_motor_accelerations(const wheels<motorVelocityType>& motor_accelerations) {
    for (int i = 0; i < 6; i++) {
	    motors[i].move_acceleration(motor_accelerations[i]);
    }
}

void drivetrain::move_voltage_time(const wheels<float>& wheel_voltages, const int time) {
    for (size_t i = 0; i < time / timestep; i++) {
        move_voltage(wheel_voltages);
	    pros::delay(timestep * 1000.f);
    }
}

wheels<wheel_vel_bounds> drivetrain::get_wheel_vel_bounds() {
    wheels<wheel_vel_bounds> result{};
    for (size_t i = 0; i < 6; ++i) {
	    result[i].min = motors[i].get_vel_bounds().min * wheel_radius;
	    result[i].max = motors[i].get_vel_bounds().max * wheel_radius;
    }
    return result;
}

wheels<motorVelocityType> drivetrain::get_wanted_motor_accels(const wheels<motorVelocityType>& desired_motor_vels) {
    wheels<motorVelocityType> result{};
    for (size_t i = 0; i < 6; ++i) {
	    result[i].velocity = motors[i].get_desired_acceleration(desired_motor_vels[i].velocity);
		result[i].brakeMode = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
    }
    return result;
}

void drivetrain::tank_drive_control() {
    const float x_right = static_cast<float>(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    const float y_left = static_cast<float>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    // for (int j = 0; j < 6; ++j) {
	    // motors[j].update_motor_data();
    // }
    // Compute per-wheel velocity bounds given current state and time step.
    const wheels<wheel_vel_bounds> bounds = get_wheel_vel_bounds();
    const float W = trackwidth_length;
	const float L = wheelbase_length;
    float linear_velocity;
    float angular_velocity;
    // Map joystick Y to a linear velocity, scaled by forward/reverse capability.
    if (y_left > 0) {
         const float lin_max_velocity = (bounds.m1.max + bounds.m2.max + bounds.m3.max + bounds.m4.max + bounds.o1.max + bounds.o2.max) / 6.f;
	 linear_velocity = std::abs(y_left) / 127.f * lin_max_velocity;
    }
    else {
         const float lin_min_velocity = (bounds.m1.min + bounds.m2.min + bounds.m3.min + bounds.m4.min + bounds.o1.min + bounds.o2.min) / 6.f;
	 linear_velocity = std::abs(y_left) / 127.f * lin_min_velocity;
    }
    // Map joystick X to an angular velocity, again using asymmetric bounds.
    if (-x_right > 0) {
    	const float ang_max_velocity = (L+W)/24.f*(bounds.m2.max + bounds.m4.max - bounds.m1.min - bounds.m3.min) + W/12.f*(bounds.o2.max-bounds.o1.min);
	angular_velocity = std::abs(x_right) / 127.f * ang_max_velocity;
    }
    else {
    	const float ang_min_velocity = (L+W)/24.f*(bounds.m2.min + bounds.m4.min - bounds.m1.max - bounds.m3.max) + W/12.f*(bounds.o2.min-bounds.o1.max);
	angular_velocity = std::abs(x_right) / 127.f * ang_min_velocity;

    }
	// Desired chassis velocity in differential form (linear, angular).
	const differentialVels robot_velocity = {linear_velocity, angular_velocity};
	const wheels<motorVelocityType> wanted_motor_vels = differential_vels_to_motor_vels(robot_velocity);
    // wheels<float> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
    const wheels<motorVelocityType> wanted_motor_accels = get_wanted_motor_accels(wanted_motor_vels);
    move_motor_accelerations(wanted_motor_accels);
}

wheels<float> drivetrain::bound_desired_motor_velocities(const wheels<float>& desired_motor_velocities) {
    wheels<float> result;
    for (size_t i = 0; i < 6; ++i) {
	    result[i] = motors[i].bound_desired_velocity(desired_motor_velocities[i]);
    }
    return result;
}

wheels<motorVelocityType> drivetrain::differential_vels_to_motor_vels(differentialVels robot_velocity) {	
    const float L = wheelbase_length;
    const float W = trackwidth_length;
	const float m1_velocity = (3.f/4.f * robot_velocity.linear - (3.f*robot_velocity.angular)/(L+W)) / wheel_radius;
	const float m2_velocity = (3.f/4.f * robot_velocity.linear + (3.f*robot_velocity.angular)/(L+W)) / wheel_radius;
	const float m3_velocity = (3.f/4.f * robot_velocity.linear - (3.f*robot_velocity.angular)/(L+W)) / wheel_radius;
	const float m4_velocity = (3.f/4.f * robot_velocity.linear + (3.f*robot_velocity.angular)/(L+W)) / wheel_radius;
	const float o1_velocity = (1.50*robot_velocity.linear-3.f*robot_velocity.angular/(W)) / wheel_radius;
	const float o2_velocity = (1.50*robot_velocity.linear+3.f*robot_velocity.angular/(W)) / wheel_radius;
	return {{m1_velocity, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {m2_velocity, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {o1_velocity, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {o2_velocity, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {m3_velocity, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {m4_velocity, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}};
}

void drivetrain::move_differential_robot_vels(const std::vector<differentialVels>& robot_vels) {
	for (size_t i = 0; i < robot_vels.size(); i++) {
		wheels<motorVelocityType> wanted_motor_vels = differential_vels_to_motor_vels(robot_vels[i]);
    	// wheels<float> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
    	const wheels<motorVelocityType> wanted_motor_accels = get_wanted_motor_accels(wanted_motor_vels);
		move_motor_accelerations(wanted_motor_accels);
	}
}

differentialVels drivetrain::ramsete(pose wanted_pose, differentialVels wanted_vels) {
	 // Compute pose error expressed in the robot frame.
    float error_theta = wrapToPi(wanted_pose.theta - localization.Pose.theta);
    float dx = wanted_pose.x - localization.Pose.x;
    float dy = wanted_pose.y - localization.Pose.y;
    float cos_t = std::cos(localization.Pose.theta);
    float sin_t = std::sin(localization.Pose.theta);
    float error_x =  sin_t * dy + cos_t * dx;
    float error_y =  cos_t * dy - sin_t * dx;

    // Ramsete gain term combining linear and angular desired speeds.
    float k2 = std::sqrt(wanted_vels.angular * wanted_vels.angular + (b_gain * wanted_vels.linear) * (b_gain * wanted_vels.linear));

    // Compute corrected outputs; see Ramsete formulation for details.
    float linear_out  = wanted_vels.linear * std::cos(error_theta) + b_gain * error_x;
    float angular_out = wanted_vels.angular + k2 * sinc(error_theta) * error_y + b_gain * error_theta;
	return {linear_out, angular_out};
}

void drivetrain::move_differential_robot_vels_ramsete(std::vector<differentialVels> robot_vels, std::vector<pose> wanted_pose) {
	for (size_t i = 0; i < robot_vels.size(); i++) {
		differentialVels corrected_robot_vels = ramsete(wanted_pose[i], robot_vels[i]); 
		wheels<motorVelocityType> wanted_motor_vels = differential_vels_to_motor_vels(corrected_robot_vels);
    	// wheels<float> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
    	const wheels<motorVelocityType> wanted_motor_accels = get_wanted_motor_accels(wanted_motor_vels);
		move_motor_accelerations(wanted_motor_accels);
        pros::delay(static_cast<int>(timestep*1000.f));
	}
}
  
void drivetrain::linear_mp(const float distance, bool log, const float percent_of_max_velocity, const float percent_of_max_acceleration) {
    float time = 0.f;
    bool start = true;
    mp linearMP(max_robot_lin_accel_scaled * percent_of_max_acceleration / 100.f, max_robot_lin_vel_scaled * percent_of_max_velocity / 100.f, distance);
	std::optional<std::vector<float>> linear_velocities;
    if (log) {
        linear_velocities.emplace();
        linear_velocities->reserve(static_cast<int>(std::round(linearMP.end_time / timestep)));
    }

    while(!linearMP.profileFinished(time) || start) {
        float linear_velocity = linearMP.velocity(time);
		if (linear_velocities) {
			linear_velocities->push_back(linear_velocity);
            // linear_velocities->push_back(get_linear_velocity());
		}
	    std::vector<differentialVels> desired_differential_vel = {{linear_velocity, 0.f}};
	    move_differential_robot_vels(desired_differential_vel);
        pros::delay(static_cast<int>(timestep * 1000.f));
        time += timestep;
        start = false;
    }

	if (linear_velocities) {
		print_vector(*linear_velocities, "L");
	}

    motor_brakes();
}


void drivetrain::angular_mp(const float angle, bool log, const float percent_of_max_velocity, const float percent_of_max_acceleration) {
    float time = 0.f;
    bool start = true;
    mp angularMP(max_robot_ang_accel_scaled * percent_of_max_acceleration / 100.f, max_robot_ang_vel_scaled * percent_of_max_velocity / 100.f, angle);
  	std::optional<std::vector<float>> angular_velocities;
    if (log) {
        angular_velocities.emplace();
        angular_velocities->reserve(static_cast<int>(std::round(angularMP.end_time / timestep)));
    }
    while(!angularMP.profileFinished(time) || start) {

        float angular_velocity = angularMP.velocity(time);
		if (angular_velocities) {
			angular_velocities->push_back(angular_velocity);
            // angular_velocities->push_back(get_angular_velocity());
		}
	    std::vector<differentialVels> desired_differential_vel = {{0.f, angular_velocity}};
	    move_differential_robot_vels(desired_differential_vel);
        pros::delay(timestep * 1000.f);
        time += timestep;
        start = false;
    }
	
	if (angular_velocities) {
		print_vector(*angular_velocities, "A");
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

float drivetrain::get_linear_velocity() {
    float total_linear_velocity = 0.f;
    for (size_t i = 0; i < 6; ++i) {
        float linear_velocity = motors[i].get_angular_velocity() * wheel_radius;
        total_linear_velocity += linear_velocity;
    }
    return total_linear_velocity / 6.f;
}

float drivetrain::get_angular_velocity() {
	const float m1_linear_velocity = motors.m1.get_angular_velocity() * wheel_radius;
	const float m2_linear_velocity = motors.m2.get_angular_velocity() * wheel_radius;
    const float o1_linear_velocity = motors.o1.get_angular_velocity() * wheel_radius;
	const float o2_linear_velocity = motors.o2.get_angular_velocity() * wheel_radius;
	const float m3_linear_velocity = motors.m3.get_angular_velocity() * wheel_radius;
	const float m4_linear_velocity = motors.m4.get_angular_velocity() * wheel_radius;
    float left_linear_velocity = (m1_linear_velocity + o1_linear_velocity + m3_linear_velocity) / 3.f;
    float right_linear_velocity = (m2_linear_velocity + o2_linear_velocity + m4_linear_velocity) / 3.f;
    return (right_linear_velocity - left_linear_velocity) / trackwidth_length;
}

void drivetrain::mtp_mp(const pose desired_pose) {
	const float desired_angle_delta = localization.Pose.theta - desired_pose.theta;
	angular_mp(desired_angle_delta);
	pros::delay(500);
	const float desired_linear_distance_delta = std::hypot(localization.Pose.x - desired_pose.x, localization.Pose.y - desired_pose.y);
	linear_mp(desired_angle_delta);
};
	
void drivetrain::mtp_mp_ramsete(const pose desired_pose) {

}
