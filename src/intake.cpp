#include "intake.hpp"

double intake::get_motor_max_accel(pros::Motor motor, const ff_constants motor_constants_) {
    double motor_velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
    double motor_voltage = motor.get_voltage() * 1000.0;
    double motor_max_acceleration = (motor_voltage - motor_velocity * motor_constants_.K_v - sgn(motor_velocity) * motor_constants_.K_s) / motor_constants_.K_a;
    double result = motor_max_acceleration;
    return result;
}

void intake::move_wheel_accels(const rollers<double>& wheel_accelerations) {
    double fb_velocity = motors.fb.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double fb_voltage = motor_ffs.fb.compute_voltage(wheel_accelerations.fb, fb_velocity) * 1000.f;
    double ft_velocity = motors.ft.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double ft_voltage = motor_ffs.ft.compute_voltage(wheel_accelerations.ft, ft_velocity) * 1000.f;
    double bb_velocity = motors.bb.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double bb_voltage = motor_ffs.bb.compute_voltage(wheel_accelerations.bb, bb_velocity) * 1000.f;
    double bt_velocity = motors.bt.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double bt_voltage = motor_ffs.bt.compute_voltage(wheel_accelerations.bt, bt_velocity) * 1000.f;
    motors.fb.get().move_voltage(static_cast<int>(std::lround(fb_voltage)));
    motors.ft.get().move_voltage(static_cast<int>(std::lround(ft_voltage)));
    motors.bb.get().move_voltage(static_cast<int>(std::lround(bb_voltage)));
    motors.bt.get().move_voltage(static_cast<int>(std::lround(bt_voltage)));
}

void intake::move_wheel_volts(const rollers<double>& wheel_voltages) {
    motors.fb.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.fb)));
    motors.ft.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.ft)));
    motors.bb.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.bb)));
    motors.bt.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.bt)));
}

rollers<wheel_vel_lim> intake::get_motor_vel_limits(const double& dt) {
    rollers<wheel_vel_lim> result{};
    result.fb.min = motors.fb.get().get_actual_velocity() * 2.f * M_PI / 60.f - motor_constants.fb.max_ang_accel * dt;
    result.ft.min = motors.ft.get().get_actual_velocity() * 2.f * M_PI / 60.f - motor_constants.ft.max_ang_accel * dt;
    result.bb.min = motors.bb.get().get_actual_velocity() * 2.f * M_PI / 60.f - motor_constants.bb.max_ang_accel * dt;
    result.bt.min = motors.bt.get().get_actual_velocity() * 2.f * M_PI / 60.f - motor_constants.bt.max_ang_accel * dt;
    result.fb.max = motors.fb.get().get_actual_velocity() * 2.f * M_PI / 60.f + motor_constants.fb.max_ang_accel * dt;
    result.ft.max = motors.ft.get().get_actual_velocity() * 2.f * M_PI / 60.f + motor_constants.ft.max_ang_accel * dt;
    result.bb.max = motors.bb.get().get_actual_velocity() * 2.f * M_PI / 60.f + motor_constants.bb.max_ang_accel * dt;
    result.bt.max = motors.bt.get().get_actual_velocity() * 2.f * M_PI / 60.f + motor_constants.bt.max_ang_accel * dt;
    return result;
}

rollers<double> intake::get_wanted_motor_accels(const rollers<double>& wanted_motor_vels, const double& dt) {
    rollers<double> result{};
    result.fb = (motors.fb.get().get_actual_velocity() * 2.f * M_PI / 60.f - wanted_motor_vels.fb) / dt;
    result.ft = (motors.ft.get().get_actual_velocity() * 2.f * M_PI / 60.f - wanted_motor_vels.ft) / dt;
    result.bb = (motors.bb.get().get_actual_velocity() * 2.f * M_PI / 60.f - wanted_motor_vels.bb) / dt;
    result.bt = (motors.bt.get().get_actual_velocity() * 2.f * M_PI / 60.f - wanted_motor_vels.bt) / dt;
    return result;
}

rollers<motorStateType> intake::get_roller_states(void) {
    rollers<motorStateType> result;
    switch (intakeState) {
        case off:
            result = {off, off, off, off};
            break;
        
	case intakeOnly:
            result = {reverse, forward, hold, forward};
            break;
        
	case bottomScore:
            result = {forward, off, reverse, off};
            break;
        
	case midScore:
            result = {reverse, reverse, reverse, forward};
            break;
	
	case topScore:
            result = {reverse, forward, reverse, forward};
            break;
    
        default:
            result = {off, off, off, off};
            break;
    }
    return result;
}

double intake::get_wanted_motor_vel(pros::Motor motor, const ff_constants motor_constants_, motorStateType wanted_roller_state, const double& dt) { 
    double motor_velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
    double motor_wanted_velocity;
    switch (wanted_roller_state) {
	case off:
	   fb_wanted_velocity = 0; 
	case forward:
	   fb_wanted_velocity = motor_constants_.max_ang_vel;
	case reverse:
	   fb_wanted_velocity = -motor_constants_.max_ang_vel;
	case hold:
	   fb_wanted_velocity = 0;
    }
    double motor_velocity_delta = fb_velocity - fb_wanted_velocity;
    double motor_wanted_velocity_bounded;
    if (fb_velocity_delta > 0) {
	double motor_max_velocity = get_motor_max_accel(motor, motor_constants_) * dt;
	motor_wanted_velocity_bounded = fmin(motor_max_velocity, motor_wanted_velocity);
    }
    else if (fb_velocity_delta < 0) {
	double motor_min_velocity = -get_motor_max_accel(motor, motor_constants_) * dt;
	motor_wanted_velocity_bounded = fmax(motor_min_velocity, motor_wanted_velocity);

    }
    else {
	motor_wanted_velocity = 0;
    }
    double wanted_motor_vels = fb_wanted_velocity_bounded;
}
