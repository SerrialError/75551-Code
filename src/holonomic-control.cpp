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
    const float L = wheelbase_length;
    const float W = trackwidth_length;
    // Core A matrix encodes chassis velocity constraints in wheel space.
    std::vector<std::vector<float>> A = {
        {1.f,         1.f,      1.f,        1.f,       1.f,    1.f  },
        {1.f,         -1.f,     -1.f,       1.f,       0.f,    0.f  },
        {-(L+W)/4.f, (L+W)/4.f, -(L+W)/4.f, (L+W)/4.f, -W/2.f, W/2.f},
        {-1.f,        -1.f,      -1.f,      -1.f,      -1.f,   -1.f },
        {-1.f,        1.f,       1.f,       -1.f,      0.f,    0.f  },
        {(L+W)/4.f, -(L+W)/4.f, (L+W)/4.f, -(L+W)/4.f, W/2.f, -W/2.f},
        {-(L+W)/4.f, (L+W)/4.f, -(L+W)/4.f, (L+W)/4.f, -W/2.f, W/2.f},
        {(L+W)/4.f, -(L+W)/4.f, (L+W)/4.f, -(L+W)/4.f, W/2.f, -W/2.f}
    };
    std::vector<float> b = {desired_vels.y, desired_vels.x, desired_vels.theta, -desired_vels.y, -desired_vels.x, -desired_vels.theta, 0.f, 0.f};

    // Add bounds: F_i <= max, -F_i <= -min for each wheel.
    std::vector<wheel_vel_bounds> lims = {
        bounds.m1, bounds.m2, bounds.m3,
        bounds.m4, bounds.o1, bounds.o2
    };

    for (int i = 0; i < n; i++) {
        std::vector<float> row(n, 0.f);
        row[i] = 1.f; A.push_back(row); b.push_back(lims[i].max);
        row[i] = -1.f; A.push_back(row); b.push_back(-lims[i].min);
    }

    // Map solution into wheel velocity bounds by solving n LPs: one per wheel.
    std::vector<float> max_result(n, 0.f);
    for (int j = 0; j < n; j++) {
        std::vector<float> c(n, 0.f);
        c[j] = 1;
        std::optional<std::vector<float>> optimal = Simplex::solve(A, b, c);
        if (!optimal) {
            max_result[j] = 0.f;
            printf("Solver error");
            // Optional: take recovery action or safely stop
        }  
        else {
            std::vector<float> sol = *optimal;
            max_result[j] = sol[j];
        }
        
    }
    std::vector<float> min_result(n, 0.f);
    for (int j = 0; j < n; j++) {
        std::vector<float> c(n, 0.f);
        c[j] = -1.f;
        std::optional<std::vector<float>> optimal = Simplex::solve(A, b, c);
        if (!optimal) {
            min_result[j] = 0.f;
            printf("Solver error");
            // Optional: take recovery action or safely stop
        }  
        else {
            std::vector<float> sol = *optimal;
            min_result[j] = sol[j];
        }
    }
    wheels<wheel_vel_bounds> result = {{min_result[0], max_result[0]}, {min_result[1], max_result[1]}, {min_result[4], max_result[4]}, {min_result[5], max_result[5]}, {min_result[2], max_result[2]}, {min_result[3], max_result[3]}};
    return result;
}

std::optional<wheels<float>> drivetrain::calculate_wheel_vels(const pose& desired_vels, const wheels<wheel_vel_bounds>& bounds) {
	const int n = 6;
	const int n2 = 2 * n;

	const std::vector<float> x_nom = {static_cast<float>(motors.m1.motor.get().get_actual_velocity()) * 2.f * static_cast<float>(static_cast<float>(M_PI)) / 60.f * wheel_radius, static_cast<float>(motors.m2.motor.get().get_actual_velocity()) * 2.f * static_cast<float>(M_PI) / 60.f * wheel_radius, static_cast<float>(motors.m3.motor.get().get_actual_velocity()) * 2.f * static_cast<float>(M_PI) / 60.f * wheel_radius, static_cast<float>(motors.m4.motor.get().get_actual_velocity()) * 2.f * static_cast<float>(M_PI) / 60.f * wheel_radius, static_cast<float>(motors.o1.motor.get().get_actual_velocity()) * 2.f * static_cast<float>(M_PI) / 60.f * wheel_radius, static_cast<float>(motors.o2.motor.get().get_actual_velocity()) * 2.f * static_cast<float>(M_PI) / 60.f * wheel_radius};

    const float L = wheelbase_length;
    const float W = trackwidth_length;
    std::vector<std::vector<float>> A = {
        {1.f,         1.f,       1.f,         1.f,       1.f,    1.f,    0.f,  0.f,  0.f,  0.f,  0.f,  0.f},
        {-1.f,        1.f,       1.f,         -1.f,      0.f,    0.f,    0.f,  0.f,  0.f,  0.f,  0.f,  0.f},
        {-(L+W)/4.f,  (L+W)/4.f, -(L+W)/4.f, (L+W)/4.f,  -W/2.f, W/2.f,  0.f,  0.f,  0.f,  0.f,  0.f,  0.f},
        {-1.f,        -1.f,      -1.f,       -1.f,       -1.f,   -1.f,   0.f,  0.f,  0.f,  0.f,  0.f,  0.f},
        {1.f,         -1.f,      -1.f,       1.f,        0.f,    0.f,    0.f,  0.f,  0.f,  0.f,  0.f,  0.f},
        {(L+W)/4.f,  -(L+W)/4.f, (L+W)/4.f,  -(L+W)/4.f, W/2.f,  -W/2.f, 0.f,  0.f,  0.f,  0.f,  0.f,  0.f},
        {-(L+W)/4.f, (L+W)/4.f,  -(L+W)/4.f, (L+W)/4.f,  -W/2.f, W/2.f,  0.f,  0.f,  0.f,  0.f,  0.f,  0.f},
        {(L+W)/4.f,  -(L+W)/4.f, (L+W)/4.f,  -(L+W)/4.f, W/2.f,  -W/2.f, 0.f,  0.f,  0.f,  0.f,  0.f,  0.f}
		
    };
    std::vector<float> b = {desired_vels.y, desired_vels.x, desired_vels.theta, -desired_vels.y, -desired_vels.x, -desired_vels.theta, 0.f, 0.f};
	for (int j = 0; j < n; ++j) {
		std::vector<float> row(n2, 0.f);
		row[n + j] = -1.f;  // -d_j
		A.push_back(row);
		b.push_back(0.f);
	}

    // Add bounds: F_i <= max, -F_i <= -min
    std::vector<wheel_vel_bounds> lims = {
        bounds.m1,  bounds.m2,  bounds.m3,
        bounds.m4,  bounds.o1,  bounds.o2,
		{0.f, 0.f}, {0.f, 0.f}, {0.f, 0.f},
		{0.f, 0.f}, {0.f, 0.f}, {0.f, 0.f}
    };

    for (int i = 0; i < n; i++) {
        std::vector<float> row(n2, 0.f);
        row[i] = 1.f; A.push_back(row); b.push_back(lims[i].max);
        row[i] = -1.f; A.push_back(row); b.push_back(-lims[i].min);
    }

	// 2) for each j: x_j - d_j <= x_nom_j  => [ e_j , -e_j ] <= x_nom_j
	for (int j = 0; j < n; ++j) {
		std::vector<float> row(n2, 0.f);
		row[j] = 1.f;         // x_j
		row[n + j] = -1.f;    // -d_j
		A.push_back(row);
		b.push_back(x_nom[j]);
		// -x_j - d_j <= -x_nom_j
		std::vector<float> row2(n2, 0.f);
		row2[j] = -1.f;
		row2[n + j] = -1.f;
		A.push_back(row2);
		b.push_back(-x_nom[j]);
	}

	// Objective: minimize sum d_j  => maximize -sum d_j
	std::vector<float> c(n2, 0.f);
	for (int j = 0; j < n; ++j) c[n + j] = -1.f;

	std::optional<std::vector<float>> sol;
    sol = Simplex::solve(A, b, c); // This might throw
    if (!sol) {
        // Handle the exception here
        printf("Solver error");
        return std::nullopt;
        // Optional: take recovery action or safely stop
    }
    std::vector<float> optimal = *sol;

    wheels<float> result = {optimal[0] / wheel_radius, optimal[1] / wheel_radius, optimal[4] / wheel_radius, optimal[5] / wheel_radius, optimal[2] / wheel_radius, optimal[3] / wheel_radius};
	return result;
}

void drivetrain::field_oriented_holonomic_control(const float dt) {
    const float L = wheelbase_length;
    const float W = trackwidth_length;
    const float ZERO_DEADBAND = 1.f;
    float y_right = static_cast<float>(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
	if (std::abs(y_right) < ZERO_DEADBAND) {
		y_right = 0.f;
	}
    float x_right = static_cast<float>(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
	if (std::abs(x_right) < ZERO_DEADBAND) {
		x_right = 0.f;
	}
    float y_left = static_cast<float>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
	if (std::abs(y_left) < ZERO_DEADBAND) {
		y_left = 0.f;
	}
    float x_left = static_cast<float>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
	if (std::abs(x_left) < ZERO_DEADBAND) {
		x_left = 0.f;
	}
    float wanted_angle = atan2(y_right, x_right);
    float cur_angle = DEG_TO_RAD_NORM(localization.Pose.theta);
    // float angle_error = wrapToPi(wanted_angle - cur_angle);
    // float angle_error = wanted_angle;
    float angle_error = 0.f;
    wheels<wheel_vel_bounds> bounds = get_wheel_vel_bounds(dt);
	float x_max_velocity = bounds.m1.max - bounds.m2.min - bounds.m3.min + bounds.m4.max;
	float y_max_velocity = bounds.m1.max + bounds.m2.max + bounds.m3.max + bounds.m4.max + bounds.o1.max + bounds.o2.max;
    float theta_max_velocity = (L+W)/4.f*(bounds.m2.max + bounds.m4.max - bounds.m1.min - bounds.m3.min) + W/2.f*(bounds.o2.max - bounds.o1.min); // THESE SHOULD BE FORCES AND TORQUES
    float theta_vel = angle_error * theta_max_velocity / static_cast<float>(static_cast<float>(M_PI));
    float x_vel = x_left * (x_max_velocity) / 127.f;
    float y_vel = y_left * (y_max_velocity) / 127.f;
    pose wanted_vels = {x_vel, y_vel, theta_vel};
    std::optional<wheels<float>> wanted_motor_vels_;
    if (wanted_vels.x == 0.f && wanted_vels.y == 0.f && wanted_vels.theta == 0.f) {
        wanted_motor_vels_ = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
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
    wheels<float> wanted_motor_vels = *wanted_motor_vels_;
    // wheels<float> wanted_bounded_motor_vels = bound_desired_motor_velocities(wanted_motor_vels, dt);
    const wheels<motorVelocityType> wanted_motor_accels = get_wanted_motor_accels({{wanted_motor_vels.m1, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {wanted_motor_vels.m2, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {wanted_motor_vels.o1, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {wanted_motor_vels.o2, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {wanted_motor_vels.m3, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}, {wanted_motor_vels.m4, pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST}}, dt);
    move_motor_accelerations(wanted_motor_accels);
}
