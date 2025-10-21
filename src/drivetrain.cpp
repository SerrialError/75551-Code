#include "drivetrain.hpp"
#include "simplex.hpp"

wheels<wheel_vel_lim> drivetrain::calculate_wheel_vels_bounds(const pose& desired_vels, const wheels<wheel_vel_lim>& in_bounds) {
    // Map pose fields to the variables used in the derivation
    const double f_x   = desired_vels.x;
    const double f_y   = desired_vels.y;
    const double tau_r = desired_vels.theta;
	const double L = wheelbase_length;
	const double W = trackwidth_length;

    // Derived constants
    const double K     = 2.0 * tau_r / (L + W); // m2+m4-m1-m3
    const double D     = tau_r / L;             // o2 - o1
    const double alpha = (K + f_x) / 2.0;       // m4 - m3
    const double beta  = (f_x - K) / 2.0;       // m1 - m2

    // Input o-bounds
    const double o1_in_min = in_bounds.o1.min;
    const double o1_in_max = in_bounds.o1.max;
    const double o2_in_min = in_bounds.o2.min;
    const double o2_in_max = in_bounds.o2.max;

    // Feasible o1 interval from intersection:
    // o1 in [ o_min1, o_max1 ] and o2 = o1 + D in [ o_min2, o_max2 ]
    const double o1_low_candidate  = std::max(o1_in_min, o2_in_min - D);
    const double o1_high_candidate = std::min(o1_in_max, o2_in_max - D);

    // If no feasible o1 interval, return input bounds (no feasible solution)
    if (o1_low_candidate > o1_high_candidate) {
        return in_bounds;
    }

    // We'll evaluate feasibility at the endpoints of the o1 interval and aggregate results.
    const double o1_candidates[2] = { o1_low_candidate, o1_high_candidate };

    // Prepare output with reversed infinities so we can aggregate mins/maxes.
    wheels<wheel_vel_lim> out;
    auto set_inf = [&](wheel_vel_lim &w) {
        w.min =  std::numeric_limits<double>::infinity();
        w.max = -std::numeric_limits<double>::infinity();
    };
    set_inf(out.m1); set_inf(out.m2); set_inf(out.o1); set_inf(out.o2);
    set_inf(out.m3); set_inf(out.m4);

    bool found_any_feasible = false;

    // Helper: original m bounds
    const double m1_in_min = in_bounds.m1.min;
    const double m1_in_max = in_bounds.m1.max;
    const double m2_in_min = in_bounds.m2.min;
    const double m2_in_max = in_bounds.m2.max;
    const double m3_in_min = in_bounds.m3.min;
    const double m3_in_max = in_bounds.m3.max;
    const double m4_in_min = in_bounds.m4.min;
    const double m4_in_max = in_bounds.m4.max;

    for (double o1_cand : o1_candidates) {
        double o2_cand = o1_cand + D;
        // S_m = m1 + m2 + m3 + m4 = f_y - o1 - o2
        double S_m = f_y - o1_cand - o2_cand; // = f_y - 2*o1_cand - D

        // Build P and Q bounds from individual m bounds and alpha,beta
        // P = m1 + m2 ; Q = m3 + m4 ; P+Q = S_m

        // P constraints from m1,m2:
        double P_min_from_m1 = 2.0 * m1_in_min - beta;
        double P_max_from_m1 = 2.0 * m1_in_max - beta;
        double P_min_from_m2 = 2.0 * m2_in_min + beta;
        double P_max_from_m2 = 2.0 * m2_in_max + beta;

        double P_min = std::max(P_min_from_m1, P_min_from_m2);
        double P_max = std::min(P_max_from_m1, P_max_from_m2);

        // Q constraints from m3,m4:
        double Q_min_from_m3 = 2.0 * m3_in_min + alpha;
        double Q_max_from_m3 = 2.0 * m3_in_max + alpha;
        double Q_min_from_m4 = 2.0 * m4_in_min - alpha;
        double Q_max_from_m4 = 2.0 * m4_in_max - alpha;

        double Q_min = std::max(Q_min_from_m3, Q_min_from_m4);
        double Q_max = std::min(Q_max_from_m3, Q_max_from_m4);

        // Intersect P with the constraint P = S_m - Q, so:
        // S_m - Q_max <= P <= S_m - Q_min
        double P_low  = std::max(P_min, S_m - Q_max);
        double P_high = std::min(P_max, S_m - Q_min);

        // If infeasible for this o1 candidate, skip
        if (P_low > P_high) {
            continue;
        }

        // Now compute ranges for m_n from P_low/P_high (affine maps)
        double m1_low = (P_low + beta) / 2.0;
        double m1_high= (P_high + beta) / 2.0;

        double m2_low = (P_low - beta) / 2.0;
        double m2_high= (P_high - beta) / 2.0;

        // Q = S_m - P ; m3 = (Q - alpha)/2 = (S_m - P - alpha)/2
        double m3_low = (S_m - P_high - alpha) / 2.0;
        double m3_high= (S_m - P_low  - alpha) / 2.0;

        double m4_low = (S_m - P_high + alpha) / 2.0;
        double m4_high= (S_m - P_low  + alpha) / 2.0;

        // Clip each to the input bounds (conservative intersection)
        m1_low = std::max(m1_low, m1_in_min);  m1_high = std::min(m1_high, m1_in_max);
        m2_low = std::max(m2_low, m2_in_min);  m2_high = std::min(m2_high, m2_in_max);
        m3_low = std::max(m3_low, m3_in_min);  m3_high = std::min(m3_high, m3_in_max);
        m4_low = std::max(m4_low, m4_in_min);  m4_high = std::min(m4_high, m4_in_max);

        // If clipping made any variable infeasible, skip this candidate
        if (m1_low > m1_high || m2_low > m2_high || m3_low > m3_high || m4_low > m4_high) {
            continue;
        }

        // Aggregate results across candidates
        out.m1.min = std::min(out.m1.min, m1_low);
        out.m1.max = std::max(out.m1.max, m1_high);

        out.m2.min = std::min(out.m2.min, m2_low);
        out.m2.max = std::max(out.m2.max, m2_high);

        out.m3.min = std::min(out.m3.min, m3_low);
        out.m3.max = std::max(out.m3.max, m3_high);

        out.m4.min = std::min(out.m4.min, m4_low);
        out.m4.max = std::max(out.m4.max, m4_high);

        // o1 and o2 ranges (conservative): will be the intersection we computed earlier,
        // but aggregate as well (use clipped values)
        double o1_clipped_low  = std::max(o1_cand, o1_in_min);
        double o1_clipped_high = std::min(o1_cand, o1_in_max);
        // Note: because we used candidates that are endpoints, o1_clipped_low==o1_clipped_high here;
        // aggregation below produces the full [o1_low_candidate, o1_high_candidate] intersection.
        out.o1.min = std::min(out.o1.min, o1_clipped_low);
        out.o1.max = std::max(out.o1.max, o1_clipped_high);

        double o2_clipped_low  = std::max(o2_cand, o2_in_min);
        double o2_clipped_high = std::min(o2_cand, o2_in_max);
        out.o2.min = std::min(out.o2.min, o2_clipped_low);
        out.o2.max = std::max(out.o2.max, o2_clipped_high);

        found_any_feasible = true;
    }

    // If nothing feasible at endpoints (rare if input is consistent), return input bounds as fallback.
    if (!found_any_feasible) {
        return in_bounds;
    }

    // After aggregation, it's possible some mins are +inf (if e.g. only o-aggregation produced values).
    // Replace any still-inf mins/max with sensible clipped values (fallback to input bounds).
    auto sanitize = [&](wheel_vel_lim &out_w, const wheel_vel_lim &in_w) {
        if (!std::isfinite(out_w.min) || !std::isfinite(out_w.max) || out_w.min > out_w.max) {
            out_w.min = in_w.min;
            out_w.max = in_w.max;
        } else {
            // final defensive clip into original input bounds
            out_w.min = std::max(out_w.min, in_w.min);
            out_w.max = std::min(out_w.max, in_w.max);
            if (out_w.min > out_w.max) { // if clip broke it, fallback
                out_w.min = in_w.min;
                out_w.max = in_w.max;
            }
        }
    };

    sanitize(out.m1, in_bounds.m1);
    sanitize(out.m2, in_bounds.m2);
    sanitize(out.m3, in_bounds.m3);
    sanitize(out.m4, in_bounds.m4);
    sanitize(out.o1, in_bounds.o1);
    sanitize(out.o2, in_bounds.o2);

    return out;
}

wheels<double> drivetrain::calculate_wheel_vels(const wheels<wheel_vel_lim>& bounds) {
	wheels<double> result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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
    const double left_voltage = y_left / 127.0 * 12.0 * 1000.0;
    const double right_voltage = y_right / 127.0 * 12.0 * 1000.0;
    const wheels<double> wanted_motor_voltages = {left_voltage, right_voltage, left_voltage, right_voltage, left_voltage, right_voltage};
    move_wheel_volts(wanted_motor_voltages);
}
