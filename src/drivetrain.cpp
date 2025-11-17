#include "drivetrain.hpp"
#include "helper-functions.hpp"
#include "simplex.hpp"
#include "structs.hpp"

wheels<wheel_vel_bounds> drivetrain::calculate_wheel_vels_bounds(const pose& desired_vels, const wheels<wheel_vel_bounds>& bounds) {
    // Decision vars: [m1, m2, m3, m4, o1, o2]
    const int n = 6;
    const double L = wheelbase_length;
    const double W = trackwidth_length;
    std::vector<std::vector<double>> A = {
        {1,         1,      1,        1,       1,    1  },
        {1,         -1,     -1,       1,       0,    0  },
        {-(L+W)/4, (L+W)/4, -(L+W)/4, (L+W)/4, -W/2, W/2},
        {-1,        -1,      -1,      -1,      -1,   -1 },
        {-1,        1,       1,       -1,      0,    0  },
        {(L+W)/4, -(L+W)/4, (L+W)/4, -(L+W)/4, W/2, -W/2},
        {-(L+W)/4, (L+W)/4, -(L+W)/4, (L+W)/4, -W/2, W/2},
        {(L+W)/4, -(L+W)/4, (L+W)/4, -(L+W)/4, W/2, -W/2}
    };
    std::vector<double> b = {desired_vels.y, desired_vels.x, desired_vels.theta, -desired_vels.y, -desired_vels.x, -desired_vels.theta, 0.0, 0.0};

    // Add bounds: F_i <= max, -F_i <= -min
    std::vector<wheel_vel_bounds> lims = {
        bounds.m1, bounds.m2, bounds.m3,
        bounds.m4, bounds.o1, bounds.o2
    };

    for (int i = 0; i < n; i++) {
        std::vector<double> row(n, 0.0);
        row[i] = 1.0; A.push_back(row); b.push_back(lims[i].max);
        row[i] = -1.0; A.push_back(row); b.push_back(-lims[i].min);
    }

    // Map solution into wheels vels
    std::vector<double> max_result(n, 0.f);
    for (int j = 0; j < n; j++) {
        std::vector<double> c(n, 0.f);
        c[j] = 1;
        std::optional<std::vector<double>> optimal = Simplex::solve(A, b, c);
        if (!optimal) {
            max_result[j] = 0.0;
            printf("Solver error");
            // Optional: take recovery action or safely stop
        }  
        else {
            std::vector<double> sol = *optimal;
            max_result[j] = sol[j];
        }
        
    }
    std::vector<double> min_result(n, 0.f);
    for (int j = 0; j < n; j++) {
        std::vector<double> c(n, 0.f);
        c[j] = -1;
        std::optional<std::vector<double>> optimal = Simplex::solve(A, b, c);
        if (!optimal) {
            min_result[j] = 0.0;
            printf("Solver error");
            // Optional: take recovery action or safely stop
        }  
        else {
            std::vector<double> sol = *optimal;
            min_result[j] = sol[j];
        }
    }
    wheels<wheel_vel_bounds> result = {{min_result[0], max_result[0]}, {min_result[1], max_result[1]}, {min_result[4], max_result[4]}, {min_result[5], max_result[5]}, {min_result[2], max_result[2]}, {min_result[3], max_result[3]}};
    return result;
}

std::optional<wheels<double>> drivetrain::calculate_wheel_vels(const pose& desired_vels, const wheels<wheel_vel_bounds>& bounds) {
	const int n = 6;
	const int n2 = 2 * n;

	const std::vector<double> x_nom = {motors.m1.motor.get().get_actual_velocity() * 2.0 * M_PI / 60.0 * wheel_radius, motors.m2.motor.get().get_actual_velocity() * 2.0 * M_PI / 60.0 * wheel_radius, motors.m3.motor.get().get_actual_velocity() * 2.0 * M_PI / 60.0 * wheel_radius, motors.m4.motor.get().get_actual_velocity() * 2.0 * M_PI / 60.0 * wheel_radius, motors.o1.motor.get().get_actual_velocity() * 2.0 * M_PI / 60.0 * wheel_radius, motors.o2.motor.get().get_actual_velocity() * 2.0 * M_PI / 60.0 * wheel_radius};

    const double L = wheelbase_length;
    const double W = trackwidth_length;
    std::vector<std::vector<double>> A = {
        {1,         1,       1,         1,       1,    1,    0,  0,  0,  0,  0,  0},
        {-1,        1,       1,         -1,      0,    0,    0,  0,  0,  0,  0,  0},
        {-(L+W)/4,  (L+W)/4, -(L+W)/4, (L+W)/4,  -W/2, W/2,  0,  0,  0,  0,  0,  0},
        {-1,        -1,      -1,       -1,       -1,   -1,   0,  0,  0,  0,  0,  0},
        {1,         -1,      -1,       1,        0,    0,    0,  0,  0,  0,  0,  0},
        {(L+W)/4,  -(L+W)/4, (L+W)/4,  -(L+W)/4, W/2,  -W/2, 0,  0,  0,  0,  0,  0},
        {-(L+W)/4, (L+W)/4,  -(L+W)/4, (L+W)/4,  -W/2, W/2,  0,  0,  0,  0,  0,  0},
        {(L+W)/4,  -(L+W)/4, (L+W)/4,  -(L+W)/4, W/2,  -W/2, 0,  0,  0,  0,  0,  0}
		
    };
    std::vector<double> b = {desired_vels.y, desired_vels.x, desired_vels.theta, -desired_vels.y, -desired_vels.x, -desired_vels.theta, 0.0, 0.0};
	for (int j = 0; j < n; ++j) {
		std::vector<double> row(n2, 0.0);
		row[n + j] = -1.0;  // -d_j
		A.push_back(row);
		b.push_back(0.0);
	}

    // Add bounds: F_i <= max, -F_i <= -min
    std::vector<wheel_vel_bounds> lims = {
        bounds.m1,  bounds.m2,  bounds.m3,
        bounds.m4,  bounds.o1,  bounds.o2,
		{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
		{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}
    };

    for (int i = 0; i < n; i++) {
        std::vector<double> row(n2, 0.0);
        row[i] = 1.0; A.push_back(row); b.push_back(lims[i].max);
        row[i] = -1.0; A.push_back(row); b.push_back(-lims[i].min);
    }

	// 2) for each j: x_j - d_j <= x_nom_j  => [ e_j , -e_j ] <= x_nom_j
	for (int j = 0; j < n; ++j) {
		std::vector<double> row(n2, 0.0);
		row[j] = 1.0;         // x_j
		row[n + j] = -1.0;    // -d_j
		A.push_back(row);
		b.push_back(x_nom[j]);
		// -x_j - d_j <= -x_nom_j
		std::vector<double> row2(n2, 0.0);
		row2[j] = -1.0;
		row2[n + j] = -1.0;
		A.push_back(row2);
		b.push_back(-x_nom[j]);
	}

	// Objective: minimize sum d_j  => maximize -sum d_j
	std::vector<double> c(n2, 0.0);
	for (int j = 0; j < n; ++j) c[n + j] = -1.0;

	std::optional<std::vector<double>> sol;
    sol = Simplex::solve(A, b, c); // This might throw
    if (!sol) {
        // Handle the exception here
        printf("Solver error");
        return std::nullopt;
        // Optional: take recovery action or safely stop
    }
    std::vector<double> optimal = *sol;

    wheels<double> result = {optimal[0] / wheel_radius, optimal[1] / wheel_radius, optimal[4] / wheel_radius, optimal[5] / wheel_radius, optimal[2] / wheel_radius, optimal[3] / wheel_radius};
	return result;
}

void drivetrain::move_motor_accelerations(const wheels<double>& motor_accelerations) {
    for (int i = 0; i < 6; i++) {
	motors[i].move_motor_acceleration(motor_accelerations[i]);
    }
}

double drivetrain::get_motor_max_accel(pros::Motor& motor, const ff_constants motor_constants_, int direction) {
    double motor_velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
    double motor_max_acceleration = 0.0;
    if (motor_velocity == 0.0) {
        motor_max_acceleration = (motor_constants_.max_voltage - motor_velocity * motor_constants_.K_v - direction * motor_constants_.K_s) / motor_constants_.K_a * 0.9;
    }
    else {
        motor_max_acceleration = (motor_constants_.max_voltage - motor_velocity * motor_constants_.K_v - sign(motor_velocity) * motor_constants_.K_s) / motor_constants_.K_a * 0.9;
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

void drivetrain::field_oriented_holonomic_control(const double& dt) {
    const double L = wheelbase_length;
    const double W = trackwidth_length;
    const double ZERO_DEADBAND = 1.0;
    double y_right = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
	if (std::abs(y_right) < ZERO_DEADBAND) {
		y_right = 0.0;
	}
    double x_right = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
	if (std::abs(x_right) < ZERO_DEADBAND) {
		x_right = 0.0;
	}
    double y_left = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
	if (std::abs(y_left) < ZERO_DEADBAND) {
		y_left = 0.0;
	}
    double x_left = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
	if (std::abs(x_left) < ZERO_DEADBAND) {
		x_left = 0.0;
	}
    double wanted_angle = atan2(y_right, x_right);
    double cur_angle = DEG_TO_RAD_NORM(localization.Pose.theta);
    // double angle_error = wrapToPi(wanted_angle - cur_angle);
    // double angle_error = wanted_angle;
    double angle_error = 0.0;
    wheels<wheel_vel_bounds> bounds = get_wheel_vel_bounds(dt);
	double x_max_velocity = bounds.m1.max - bounds.m2.min - bounds.m3.min + bounds.m4.max;
	double y_max_velocity = bounds.m1.max + bounds.m2.max + bounds.m3.max + bounds.m4.max + bounds.o1.max + bounds.o2.max;
    double theta_max_velocity = (L+W)/4.0*(bounds.m2.max + bounds.m4.max - bounds.m1.min - bounds.m3.min) + W/2.0*(bounds.o2.max - bounds.o1.min); // THESE SHOULD BE FORCES AND TORQUES
    double theta_vel = angle_error * theta_max_velocity / M_PI;
    double x_vel = x_left * (x_max_velocity) / 127.0;
    double y_vel = y_left * (y_max_velocity) / 127.0;
    pose wanted_vels = {x_vel, y_vel, theta_vel};
    std::optional<wheels<double>> wanted_motor_vels_;
    if (wanted_vels.x == 0 && wanted_vels.y == 0 && wanted_vels.theta == 0) {
        wanted_motor_vels_ = {0, 0, 0, 0, 0, 0};
    }
    else {
        wanted_motor_vels_ = calculate_wheel_vels(wanted_vels, bounds);
    }
    if (!wanted_motor_vels_) {
        printf("\nSolver error\n");
        printf("x_vel: %lf", x_vel);
        printf("\ny_vel: %lf", y_vel);
        return;
    }
    wheels<double> wanted_motor_vels = *wanted_motor_vels_;
    wheels<double> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
    const wheels<double> wanted_motor_accels = get_wanted_motor_accels(wanted_bounded_motor_vels, dt);
    move_motor_accelerations(wanted_motor_accels);
}

wheels<wheel_vel_bounds> drivetrain::get_wheel_vel_bounds(const double& dt) {
    wheels<wheel_vel_bounds> result{};
    for (int i = 0; i < 6; ++i) {
	result[i] = motors[i].get_motor_vel_bounds(dt);
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
    const wheels<wheel_vel_bounds> bounds = get_wheel_vel_bounds(dt);
    const double W = trackwidth_length;
	const double L = wheelbase_length;
    const double lin_max_velocity = (bounds.m1.max + bounds.m2.max + bounds.m3.max + bounds.m4.max + bounds.o1.max + bounds.o2.max) / 6.0;
    const double ang_max_velocity = (L+W)/24.0*(bounds.m2.max + bounds.m4.max - bounds.m1.min - bounds.m3.min) + W/12.0*(bounds.o2.max-bounds.o1.min);
	const double linear_velocity = y_left / 127.0 * lin_max_velocity;
	const double angular_velocity = -x_right / 127.0 * ang_max_velocity;
	const differentialVels robot_velocity = {linear_velocity, angular_velocity};
	const wheels<double> wanted_motor_vels = differential_vels_to_motor_vels(robot_velocity);
    wheels<double> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
    const wheels<double> wanted_motor_accels = get_wanted_motor_accels(wanted_bounded_motor_vels, dt);
    move_motor_accelerations(wanted_motor_accels);
}

double drivetrain::get_wanted_motor_vel(pros::Motor& motor, const ff_constants motor_constants_, const double wanted_velocity, const double& dt) {
    double velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
	double max_velocity_change = get_motor_max_accel(motor, motor_constants_, sign(wanted_velocity)) * dt;
    double max_velocity = velocity + max_velocity_change;
    double min_velocity = velocity - max_velocity_change;
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
    		wheels<double> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
		const wheels<double> wanted_motor_accels = get_wanted_motor_accels(wanted_bounded_motor_vels, dt);
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

void drivetrain::linear_mp(const double distance) {
    double time = 0.0;
    while(!LinearMP.profileFinished(time)) {
        double linear_velocity = LinearMP.velocity(time, distance);
	std::vector<differentialVels> desired_differential_vel = {{linear_velocity, 0.0}};
	move_differential_robot_vels(desired_differential_vel, 100.0);
        pros::delay(10);
        time += 0.01;
    }
    
}
