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

void intake::move_wheel_accels(const rollers<motorVelocityType>& wheel_accelerations) {
    double fb_velocity = motors.fb.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double fb_voltage = motor_ffs.fb.compute_voltage(wheel_accelerations.fb.velocity, fb_velocity) * 1000.0;
    double ft_velocity = motors.ft.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double ft_voltage = motor_ffs.ft.compute_voltage(wheel_accelerations.ft.velocity, ft_velocity) * 1000.0;
    double bb_velocity = motors.bb.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double bb_voltage = motor_ffs.bb.compute_voltage(wheel_accelerations.bb.velocity, bb_velocity) * 1000.0;
    double bt_velocity = motors.bt.get().get_actual_velocity() * 2.0 * M_PI / 60.0;
    double bt_voltage = motor_ffs.bt.compute_voltage(wheel_accelerations.bt.velocity, bt_velocity) * 1000.0;
    if (fb_voltage == 0.0 && wheel_accelerations.fb.brakeMode == pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD) {
        motors.fb.get().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        motors.fb.get().brake();
    }
    else {
        motors.fb.get().move_voltage(static_cast<int>(std::lround(fb_voltage)));
    }

    if (ft_voltage == 0.0 && wheel_accelerations.ft.brakeMode == pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD) {
        motors.ft.get().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        motors.ft.get().brake();
    }
    else {
        motors.ft.get().move_voltage(static_cast<int>(std::lround(ft_voltage)));
    }
    
    if (bb_voltage == 0.0 && wheel_accelerations.bb.brakeMode == pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD) {
        motors.bb.get().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        motors.bb.get().brake();
    }
    else {
        motors.bb.get().move_voltage(static_cast<int>(std::lround(bb_voltage)));
    }
    
    if (bt_voltage == 0.0 && wheel_accelerations.bt.brakeMode == pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD) {
        motors.bt.get().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        motors.bt.get().brake();
    }
    else {
        motors.bt.get().move_voltage(static_cast<int>(std::lround(bt_voltage)));
    }
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
    using enum motorStateType;
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

motorVelocityType intake::get_wanted_motor_vel(pros::Motor& motor, const ff_constants motor_constants_, motorStateType wanted_roller_state, const double& dt) { 
    double velocity = motor.get_actual_velocity() * 2.0 * M_PI / 60.0;
    double wanted_velocity = 0.0;
    pros::motor_brake_mode_e wanted_brake{};
    using enum motorStateType; 
    switch (wanted_roller_state) {
	    case off:
	        wanted_velocity = 0.0;
            wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
            break;
	    case forward:
	        wanted_velocity = motor_constants_.max_ang_vel;
            wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
            break;
	    case reverse:
	        wanted_velocity = -motor_constants_.max_ang_vel;
            wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
            break;
	    case hold:
	        wanted_velocity = 0.0;
            wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD;
            break;
        default:
	        wanted_velocity = 0.0;
            wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
            break;
    }
	double max_velocity_change = get_motor_max_accel(motor, motor_constants_, sgn(wanted_velocity)) * dt;
    double max_velocity = velocity + max_velocity_change;
    double min_velocity = velocity - max_velocity_change;
	double wanted_velocity_bounded = std::clamp(wanted_velocity, min_velocity, max_velocity);
    
	const double ZERO_DEADBAND_RAD_PER_S = 1.2 * motor_constants_.K_s / motor_constants_.K_v;

	if (std::abs(wanted_velocity_bounded) < ZERO_DEADBAND_RAD_PER_S) {
		wanted_velocity_bounded = 0.0;
	}
	return {wanted_velocity_bounded, wanted_brake};
}

void intake::get_intake_state() {
    if (intakeState != intakeOff) {
        if (!is_correct_color()) {
            if (intakeState == midScore) {
                intakeState = topScore;
            }
            else {
                intakeState = midScore;
            }
        }
    }
}

void intake::update_intake_state(const double& dt) {
	if (colorSorting) {
        get_intake_state();
    }
    rollers<motorStateType> roller_states = get_roller_states();
    motorVelocityType fb_motor_velocity = get_wanted_motor_vel(motors.fb.get(), motor_constants.fb, roller_states.fb, dt);
    motorVelocityType ft_motor_velocity = get_wanted_motor_vel(motors.ft.get(), motor_constants.ft, roller_states.ft, dt);
    motorVelocityType bb_motor_velocity = get_wanted_motor_vel(motors.bb.get(), motor_constants.bb, roller_states.bb, dt);
    motorVelocityType bt_motor_velocity = get_wanted_motor_vel(motors.bt.get(), motor_constants.bt, roller_states.bt, dt);
    rollers<double> motor_velocities = {fb_motor_velocity.velocity, ft_motor_velocity.velocity, bb_motor_velocity.velocity, bt_motor_velocity.velocity};
    rollers<double> motor_accelerations = get_wanted_motor_accels(motor_velocities, dt);
    rollers<motorVelocityType> motor_accelerations_and_braking = {{motor_accelerations.fb, fb_motor_velocity.brakeMode}, {motor_accelerations.ft, ft_motor_velocity.brakeMode}, {motor_accelerations.bb, bb_motor_velocity.brakeMode}, {motor_accelerations.bt, bt_motor_velocity.brakeMode}};
    move_wheel_accels(motor_accelerations_and_braking);
}

bool intake::is_correct_color(void) {
    color blockColor = get_block_color();
	if (blockColor == allianceColor || blockColor == none) {
		return true;
	}
	else {
		return false;
	}
}

color intake::get_block_color(void) {
	double hue = optical.get_hue();
    master.print(0, 0, "%lf", hue);
    master.clear();
	double redAngularDistance = angularDistance(hue, redHue);
	double blueAngularDistance = angularDistance(hue, redHue);
	if (redAngularDistance < redHueUncertainty) {
		return red;
	}
	else if (blueAngularDistance < blueHueUncertainty) {
		return blue;
	}
	else {
		return none;
	}

}
