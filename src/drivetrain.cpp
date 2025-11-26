#include "drivetrain.hpp"
#include "helper-functions.hpp"
#include "simplex.hpp"
#include "structs.hpp"

double drivetrain::get_motor_max_accel(pros::Motor& motor, const ff_constants motor_constants_, bool reverse, int direction) {
    double motor_velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
    double motor_max_acceleration = 0.0;
    if (!reverse) {
        if (motor_velocity == 0.0) {
            motor_max_acceleration = (motor_constants_.max_voltage - motor_velocity * motor_constants_.K_v - direction * motor_constants_.K_s) / motor_constants_.K_a;
        }
        else {
            motor_max_acceleration = (motor_constants_.max_voltage - motor_velocity * motor_constants_.K_v - sign(motor_velocity) * motor_constants_.K_s) / motor_constants_.K_a;
        }
    }
    else {
        if (motor_velocity == 0.0) {
            motor_max_acceleration = (-1.0 * motor_constants_.max_voltage - motor_velocity * motor_constants_.K_v - direction * motor_constants_.K_s) / motor_constants_.K_a;
        }
        else {
            motor_max_acceleration = (-1.0 * motor_constants_.max_voltage - motor_velocity * motor_constants_.K_v - sign(motor_velocity) * motor_constants_.K_s) / motor_constants_.K_a;
        }
    }

    return motor_max_acceleration;
}

void drivetrain::move_wheel_volts(const wheels<double>& wheel_voltages) {
    for (int i = 0; i < 6; ++i) {
        motors[i].move_motor_voltage(wheel_voltages[i]);
    }
}

void drivetrain::move_wheel_volts_time(const wheels<double>& wheel_voltages, const int time) {
    for (int i = 0; i < time / 10; i++) {
        move_wheel_volts(wheel_voltages);
	    pros::delay(10);
    }
}

wheels<wheel_vel_bounds> drivetrain::get_wheel_vel_bounds(const double& dt) {
    wheels<wheel_vel_bounds> result{};
    for (int i = 0; i < 6; ++i) {
	    result[i].min = motors[i].get_motor_vel_bounds(dt).min * wheel_radius;
	    result[i].max = motors[i].get_motor_vel_bounds(dt).max * wheel_radius;
    }
    return result;
}

wheels<double> drivetrain::get_wanted_motor_accels(const wheels<double>& desired_motor_vels, const double& dt) {
    wheels<double> result{};
    for (int i = 0; i < 6; ++i) {
	    result[i] = motors[i].get_desired_motor_acceleration(desired_motor_vels[i], dt);
    }
    return result;
}

void drivetrain::tank_drive_control(const double& dt) {
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
	const wheels<double> wanted_motor_vels = differential_vels_to_motor_vels(robot_velocity);
    // wheels<double> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
    const wheels<double> wanted_motor_accels = get_wanted_motor_accels(wanted_motor_vels, dt);
    move_motor_accelerations(wanted_motor_accels);
}

double drivetrain::get_wanted_motor_vel(pros::Motor& motor, const ff_constants motor_constants_, const double wanted_velocity, const double& dt) {
    double velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
	double max_velocity_change = get_motor_max_accel(motor, motor_constants_, false, sign(wanted_velocity)) * dt;
    double min_velocity_change = get_motor_max_accel(motor, motor_constants_, true, sign(wanted_velocity)) * dt;
    double max_velocity = velocity + max_velocity_change;
    double min_velocity = velocity + min_velocity_change;
	double wanted_velocity_bounded = std::clamp(wanted_velocity, min_velocity, max_velocity);
    
	const double ZERO_DEADBAND_RAD_PER_S = 1.2 * motor_constants_.K_s / motor_constants_.K_v;

	if (std::abs(wanted_velocity_bounded) < ZERO_DEADBAND_RAD_PER_S) {
		wanted_velocity_bounded = 0.0;
	}
	return wanted_velocity_bounded;
}

wheels<double> drivetrain::bound_desired_motor_velocities(const wheels<double>& desired_motor_velocities, const double& dt) {
    wheels<double> result;
    for (int i = 0; i < 6; ++i) {
	    result[i] = motors[i].bound_desired_motor_velocity(desired_motor_velocities[i], dt);
    }
    return result;
}

wheels<double> drivetrain::differential_vels_to_motor_vels(differentialVels robot_velocity) {	
    const double L = wheelbase_length;
    const double W = trackwidth_length;
	const double m1_velocity = (3.0/4.0 * robot_velocity.linear - (3.0*robot_velocity.angular)/(L+W)) / wheel_radius;
	const double m2_velocity = (3.0/4.0 * robot_velocity.linear + (3.0*robot_velocity.angular)/(L+W)) / wheel_radius;
	const double m3_velocity = (3.0/4.0 * robot_velocity.linear - (3.0*robot_velocity.angular)/(L+W)) / wheel_radius;
	const double m4_velocity = (3.0/4.0 * robot_velocity.linear + (3.0*robot_velocity.angular)/(L+W)) / wheel_radius;
	const double o1_velocity = (1.50*robot_velocity.linear-3.0*robot_velocity.angular/(W)) / wheel_radius;
	const double o2_velocity = (1.50*robot_velocity.linear+3.0*robot_velocity.angular/(W)) / wheel_radius;
	return {m1_velocity, m2_velocity, o1_velocity, o2_velocity, m3_velocity, m4_velocity};
}

void drivetrain::move_differential_robot_vels(std::vector<differentialVels> robot_vels, const double& dt) {
	for (int i = 0; i < robot_vels.size(); i++) {
		wheels<double> wanted_motor_vels = differential_vels_to_motor_vels(robot_vels[i]);
    	// wheels<double> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
		const wheels<double> wanted_motor_accels = get_wanted_motor_accels(wanted_motor_vels, dt);
		move_motor_accelerations(wanted_motor_accels);
        pros::delay(static_cast<int>(dt*100.0));
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

void drivetrain::move_differential_robot_vels_ramsete(std::vector<differentialVels> robot_vels, std::vector<pose> wanted_pose, const double& dt) {
	for (int i = 0; i < robot_vels.size(); i++) {
		differentialVels corrected_robot_vels = ramsete(wanted_pose[i], robot_vels[i]); 
		wheels<double> wanted_motor_vels = differential_vels_to_motor_vels(corrected_robot_vels);
    	wheels<double> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
		const wheels<double> wanted_motor_accels = get_wanted_motor_accels(wanted_bounded_motor_vels, dt);
		move_motor_accelerations(wanted_motor_accels);
        pros::delay(static_cast<int>(dt*100.0));
	}
}

void drivetrain::print_motor_data() {
    for (int i = 0; i < 6; ++i) {
	    motors[i].print_vector();
        pros::delay(10);
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
    move_wheel_volts({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}
