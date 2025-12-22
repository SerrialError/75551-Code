/**
 * \file holonomic-control.cpp
 * @brief Holonomic velocity allocation using linear programming.
 *
 * Implements the methods that convert desired chassis velocities into
 * individual wheel velocity bounds and exact wheel velocity solutions
 * using a simplex-based linear program. These routines enforce per-wheel
 * limits while trying to stay close to the nominal solution.
 */
#include "drivetrain.hpp"
#include "helper-functions.hpp"
#include "simplex.hpp"
#include "structs.hpp"

wheels<wheel_vel_bounds> drivetrain::calculate_wheel_vels_bounds(const pose& desired_vels, const wheels<wheel_vel_bounds>& bounds) {
    // Decision vars: [m1, m2, m3, m4, o1, o2]
    const int n = 6;
    const double L = wheelbase_length;
    const double W = trackwidth_length;
    // Core A matrix encodes chassis velocity constraints in wheel space.
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

    // Add bounds: F_i <= max, -F_i <= -min for each wheel.
    std::vector<wheel_vel_bounds> lims = {
        bounds.m1, bounds.m2, bounds.m3,
        bounds.m4, bounds.o1, bounds.o2
    };

    for (int i = 0; i < n; i++) {
        std::vector<double> row(n, 0.0);
        row[i] = 1.0; A.push_back(row); b.push_back(lims[i].max);
        row[i] = -1.0; A.push_back(row); b.push_back(-lims[i].min);
    }

    // Map solution into wheel velocity bounds by solving n LPs: one per wheel.
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

void drivetrain::field_oriented_holonomic_control(const double dt) {
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
    // wheels<double> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
    const wheels<motorVelocityType> wanted_motor_accels = get_wanted_motor_accels({{wanted_motor_vels.m1, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {wanted_motor_vels.m2, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {wanted_motor_vels.o1, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {wanted_motor_vels.o2, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {wanted_motor_vels.m3, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {wanted_motor_vels.m4, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}}, dt);
    move_motor_accelerations(wanted_motor_accels);
}
