#include "drivetrain.hpp"
#include "simplex.hpp"

wheels<wheel_vel_lim> drivetrain::calculate_wheel_vels_limits(const pose& desired_vels, const wheels<wheel_vel_lim>& limits) {
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
    };
    std::vector<double> b = {desired_vels.y, desired_vels.x, desired_vels.theta, -desired_vels.y, -desired_vels.x, -desired_vels.theta};

    // Add bounds: F_i <= max, -F_i <= -min
    std::vector<wheel_vel_lim> lims = {
        limits.m1, limits.m2, limits.m3,
        limits.m4, limits.o1, limits.o2
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
        std::vector<double> sol = Simplex::solve(A, b, c);
        max_result[j] = sol[j];
    }
    std::vector<double> min_result(n, 0.f);
    for (int j = 0; j < n; j++) {
        std::vector<double> c(n, 0.f);
        c[j] = -1;
        std::vector<double> sol = Simplex::solve(A, b, c);
        min_result[j] = sol[j];
    }
    wheels<wheel_vel_lim> result = {{min_result[0], max_result[0]}, {min_result[1], max_result[1]}, {min_result[4], max_result[4]}, {min_result[5], max_result[5]}, {min_result[2], max_result[2]}, {min_result[3], max_result[3]}};
    return result;
}

void drivetrain::move_wheel_accels(const wheels<double>& wheel_accelerations) {
    double m1_velocity = motors.m1.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double m1_voltage = motor_ffs.m1.compute_voltage(wheel_accelerations.m1, m1_velocity) * 1000.0;
    double m2_velocity = motors.m2.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double m2_voltage = motor_ffs.m2.compute_voltage(wheel_accelerations.m2, m2_velocity) * 1000.0;
    double o1_velocity = motors.o1.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double o1_voltage = motor_ffs.o1.compute_voltage(wheel_accelerations.o1, o1_velocity) * 1000.0;
    double o2_velocity = motors.o2.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double o2_voltage = motor_ffs.o2.compute_voltage(wheel_accelerations.o2, o2_velocity) * 1000.0;
    double m3_velocity = motors.m3.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double m3_voltage = motor_ffs.m3.compute_voltage(wheel_accelerations.m3, m3_velocity) * 1000.0;
    double m4_velocity = motors.m4.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double m4_voltage = motor_ffs.m4.compute_voltage(wheel_accelerations.m4, m4_velocity) * 1000.0;
    motors.m1.get().move_voltage(static_cast<int>(std::lround(m1_voltage)));
    motors.m2.get().move_voltage(static_cast<int>(std::lround(m2_voltage)));
    motors.o1.get().move_voltage(static_cast<int>(std::lround(o1_voltage)));
    motors.o2.get().move_voltage(static_cast<int>(std::lround(o2_voltage)));
    motors.m3.get().move_voltage(static_cast<int>(std::lround(m3_voltage)));
    motors.m4.get().move_voltage(static_cast<int>(std::lround(m4_voltage)));
}

double drivetrain::get_motor_max_accel(pros::Motor motor, const ff_constants motor_constants_) {
    double motor_velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
    double motor_voltage = motor.get_voltage() * 1000.0;
    double motor_max_acceleration = (motor_voltage - motor_velocity * motor_constants_.K_v - sgn(motor_velocity) * motor_constants_.K_s) / motor_constants_.K_a;
    double result = motor_max_acceleration;
    return result;
}

void drivetrain::move_wheel_volts(const wheels<double>& wheel_voltages) {
    motors.m1.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.m1)));
    motors.m2.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.m2)));
    motors.o1.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.o1)));
    motors.o2.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.o2)));
    motors.m3.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.m3)));
    motors.m4.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.m4)));
}

void drivetrain::field_oriented_holonomic_control(const double& dt) {
    const double theta_max_velocity = 34234.434;
    const double y_right = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    const double x_right = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    const double y_left = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    const double x_left = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    double wanted_angle = atan2(y_right, x_right);
    double cur_angle = get_standard_angle();
    double angle_error = wanted_angle - cur_angle;
    double theta_vel = angle_error * theta_max_velocity / M_PI;
    double x_vel = x_left * x_max_velocity / 127.0;
    double y_vel = y_left * y_max_velocity / 127.0;
    pose wanted_vels = {x_vel, y_vel, theta_vel};
    wheels<wheel_vel_lim> motor_vel_limits = get_motor_vel_limits(dt);
    // const wheels<double> wanted_motor_vels = calculate_wheel_vels(wanted_vels, motor_vel_limits);
    // const wheels<double> wanted_motor_accels = get_wanted_motor_accels(wanted_motor_vels, dt);
    // move_wheel_accels(wanted_motor_accels);
}

double drivetrain::get_standard_angle(void) {
    double angle_rad = imu.get_rotation() * M_PI / 180.0; 
    double standard_angle = -angle_rad + M_PI;
    double offset_angle = standard_angle + initial_pose.theta;
    double clamped_angle = fmod(offset_angle + M_PI, 2.0 * M_PI) - M_PI;
    return clamped_angle;
}

wheels<wheel_vel_lim> drivetrain::get_motor_vel_limits(const double& dt) {
    wheels<wheel_vel_lim> result{};
    result.m1.min = motors.m1.get().get_actual_velocity() * 2.0 * M_PI / 60.0 - get_motor_max_accel(motors.m1.get(), motor_constants.m1) * dt;
    result.m2.min = motors.m2.get().get_actual_velocity() * 2.0 * M_PI / 60.0 - get_motor_max_accel(motors.o1.get(), motor_constants.o1) * dt;
    result.o1.min = motors.o1.get().get_actual_velocity() * 2.0 * M_PI / 60.0 - get_motor_max_accel(motors.o1.get(), motor_constants.o1) * dt;
    result.o2.min = motors.o2.get().get_actual_velocity() * 2.0 * M_PI / 60.0 - get_motor_max_accel(motors.o2.get(), motor_constants.o2) * dt;
    result.m3.min = motors.m3.get().get_actual_velocity() * 2.0 * M_PI / 60.0 - get_motor_max_accel(motors.m3.get(), motor_constants.m3) * dt;
    result.m4.min = motors.m4.get().get_actual_velocity() * 2.0 * M_PI / 60.0 - get_motor_max_accel(motors.m4.get(), motor_constants.m4) * dt;
    result.m1.max = motors.m1.get().get_actual_velocity() * 2.0 * M_PI / 60.0 + get_motor_max_accel(motors.m1.get(), motor_constants.m1) * dt;
    result.m2.max = motors.m2.get().get_actual_velocity() * 2.0 * M_PI / 60.0 + get_motor_max_accel(motors.m2.get(), motor_constants.m2) * dt;
    result.o1.max = motors.o1.get().get_actual_velocity() * 2.0 * M_PI / 60.0 + get_motor_max_accel(motors.o1.get(), motor_constants.o1) * dt;
    result.o2.max = motors.o2.get().get_actual_velocity() * 2.0 * M_PI / 60.0 + get_motor_max_accel(motors.o2.get(), motor_constants.o2) * dt;
    result.m3.max = motors.m3.get().get_actual_velocity() * 2.0 * M_PI / 60.0 + get_motor_max_accel(motors.m3.get(), motor_constants.m3) * dt;
    result.m4.max = motors.m4.get().get_actual_velocity() * 2.0 * M_PI / 60.0 + get_motor_max_accel(motors.m4.get(), motor_constants.m4) * dt;
    return result;
}

wheels<double> drivetrain::get_wanted_motor_accels(const wheels<double>& wanted_motor_vels, const double& dt) {
    wheels<double> result{};
    result.m1 = (motors.m1.get().get_actual_velocity() * 2.0 * M_PI / 60.0 - wanted_motor_vels.m1) / dt;
    result.m2 = (motors.m2.get().get_actual_velocity() * 2.0 * M_PI / 60.0 - wanted_motor_vels.m2) / dt;
    result.o1 = (motors.o1.get().get_actual_velocity() * 2.0 * M_PI / 60.0 - wanted_motor_vels.o1) / dt;
    result.o2 = (motors.o2.get().get_actual_velocity() * 2.0 * M_PI / 60.0 - wanted_motor_vels.o2) / dt;
    result.m3 = (motors.m3.get().get_actual_velocity() * 2.0 * M_PI / 60.0 - wanted_motor_vels.m3) / dt;
    result.m4 = (motors.m4.get().get_actual_velocity() * 2.0 * M_PI / 60.0 - wanted_motor_vels.m4) / dt;
    return result;
}

void drivetrain::tank_drive_control() {
    const double y_right = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    const double y_left = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    const double left_voltage = -y_left / 127.0 * 12.0 * 1000.0;
    const double right_voltage = -y_right / 127.0 * 12.0 * 1000.0;
    const wheels<double> wanted_motor_voltages = {left_voltage, right_voltage, left_voltage, right_voltage, left_voltage, right_voltage};
    move_wheel_volts(wanted_motor_voltages);
}
