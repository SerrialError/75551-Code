#include "intake.hpp"
#include <algorithm>
#include <cstdlib>

double intake::get_motor_max_accel(pros::Motor& motor, const ff_constants motor_constants_, int direction) {
    double motor_velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
    double motor_max_acceleration = 0.0;
    if (motor_velocity == 0.0) {
        motor_max_acceleration = (motor_constants_.max_voltage - motor_velocity * motor_constants_.K_v - direction * motor_constants_.K_s) / motor_constants_.K_a * 0.9;
    }
    else {
        motor_max_acceleration = (motor_constants_.max_voltage - motor_velocity * motor_constants_.K_v - sgn(motor_velocity) * motor_constants_.K_s) / motor_constants_.K_a * 0.9;
    }
    double result = motor_max_acceleration;
    return result;
}

void intake::move_wheel_accels(const rollers<double>& wheel_accelerations) {
    double fb_velocity = motors.fb.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double fb_voltage = motor_ffs.fb.compute_voltage(wheel_accelerations.fb, fb_velocity) * 1000.0;
    double ft_velocity = motors.ft.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double ft_voltage = motor_ffs.ft.compute_voltage(wheel_accelerations.ft, ft_velocity) * 1000.0;
    double bb_velocity = motors.bb.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double bb_voltage = motor_ffs.bb.compute_voltage(wheel_accelerations.bb, bb_velocity) * 1000.0;
    double bt_velocity = motors.bt.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double bt_voltage = motor_ffs.bt.compute_voltage(wheel_accelerations.bt, bt_velocity) * 1000.0;
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

rollers<double> intake::get_wanted_motor_accels(const rollers<double>& wanted_motor_vels, const double& dt) {
    rollers<double> result{};
    result.fb = (wanted_motor_vels.fb - motors.fb.get().get_actual_velocity() * 2.0 * M_PI / 60.0) / dt;
    result.ft = (wanted_motor_vels.ft - motors.ft.get().get_actual_velocity() * 2.0 * M_PI / 60.0) / dt;
    result.bb = (wanted_motor_vels.bb - motors.bb.get().get_actual_velocity() * 2.0 * M_PI / 60.0) / dt;
    result.bt = (wanted_motor_vels.bt - motors.bt.get().get_actual_velocity() * 2.0 * M_PI / 60.0) / dt;
    return result;
}

rollers<motorStateType> intake::get_roller_states(void) {
    rollers<motorStateType> result;
    switch (intakeState) {
        case intakeOff:
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

double intake::get_wanted_motor_vel(pros::Motor& motor, const ff_constants motor_constants_, motorStateType wanted_roller_state, const double& dt) { 
    double velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
    double wanted_velocity = 0.0;
    switch (wanted_roller_state) {
	    case off:
	        wanted_velocity = 0.0;
            break;
	    case forward:
	        wanted_velocity = motor_constants_.max_ang_vel;
            break;
	    case reverse:
	        wanted_velocity = -motor_constants_.max_ang_vel;
            break;
	    case hold:
	        wanted_velocity = 0.0;
            break;
        default:
	        wanted_velocity = 0.0;
            break;
    }
	double max_velocity_change = get_motor_max_accel(motor, motor_constants_, sgn(wanted_velocity)) * dt;
    printf("Current Velocity: %.6f\n", velocity);
    printf("Max Velocity Change: %.6f\n", max_velocity_change);
    printf("Wanted Velocity (END): %.6f\n", wanted_velocity);
    double max_velocity = velocity + max_velocity_change;
    double min_velocity = velocity - max_velocity_change;
	double wanted_velocity_bounded = std::clamp(wanted_velocity, min_velocity, max_velocity);
    
	const double ZERO_DEADBAND_RAD_PER_S = 1.2 * motor_constants_.K_s / motor_constants_.K_v;

	if (std::abs(wanted_velocity_bounded) < ZERO_DEADBAND_RAD_PER_S) {
		wanted_velocity_bounded = 0.0;
	}
	return wanted_velocity_bounded;
}

void intake::update_intake_state(const double& dt) {
    rollers<motorStateType> roller_states = get_roller_states();
    double fb_motor_velocity = get_wanted_motor_vel(motors.fb.get(), motor_constants.fb, roller_states.fb, dt);
    double ft_motor_velocity = get_wanted_motor_vel(motors.ft.get(), motor_constants.ft, roller_states.ft, dt);
    double bb_motor_velocity = get_wanted_motor_vel(motors.bb.get(), motor_constants.bb, roller_states.bb, dt);
    double bt_motor_velocity = get_wanted_motor_vel(motors.bt.get(), motor_constants.bt, roller_states.bt, dt);
    rollers<double> motor_velocities = {fb_motor_velocity, ft_motor_velocity, bb_motor_velocity, bt_motor_velocity};
    rollers<double> motor_accelerations = get_wanted_motor_accels(motor_velocities, dt);
    move_wheel_accels(motor_accelerations);
}
