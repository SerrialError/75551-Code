#include "intake.hpp"
#include "structs.hpp"

void intake::move_motor_accelerations(const rollers<motorVelocityType>& motor_accelerations) {
    for (int i = 0; i < 4; i++) {
	    motors[i].move_motor_acceleration(motor_accelerations[i]);
    }
}

rollers<motorVelocityType> intake::get_wanted_motor_accels(const rollers<motorVelocityType>& desired_motor_vels, const double& dt) {
    rollers<motorVelocityType> result{};
    for (size_t i = 0; i < 4; ++i) {
	    result[i].velocity = motors[i].get_desired_motor_acceleration(desired_motor_vels[i].velocity, dt);
		result[i].brakeMode = desired_motor_vels[i].brakeMode;
    }
    return result;
}

rollers<motorStateType> intake::get_roller_states(void) {
    rollers<motorStateType> result;
    using enum motorStateType;
    switch (intakeState) {
        case intakeOff:
            result = {off, off};
            break;
        
	    case intakeOnly:
            result = {forward, off};
            break;
        
	    case bottomScore:
            result = {reverse, reverse};
            break;
        
	    case midScore:
            result = {forward, reverse};
            break;
	
	    case topScore:
            result = {forward, forward};
            break;
    
        default:
            result = {off, off};
            break;
    }
    
    return result;
}

motorVelocityType intake::get_desired_motor_state(motorStateType wanted_roller_state, MotorController motor) {
    double wanted_velocity;
	pros::motor_brake_mode_e wanted_brake{};
    using enum motorStateType; 
    switch (wanted_roller_state) {
	    case off:
	        wanted_velocity = 0.0;
            wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
            break;
	    case forward:
	        wanted_velocity = motor.motor_constants.max_ang_vel;
            wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
            break;
	    case reverse:
	        wanted_velocity = -motor.motor_constants.max_ang_vel;
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
    wanted_velocity = motor.bound_velocity_to_deadband(wanted_velocity);
	return {wanted_velocity, wanted_brake};
}

rollers<motorVelocityType> intake::get_desired_motor_states(rollers<motorStateType> wanted_roller_states) {
    rollers<motorVelocityType> desired_motor_states;
	for (size_t i = 0; i < 4; ++i) {
        desired_motor_states[i] = get_desired_motor_state(wanted_roller_states[i], motors[i]);
    }
	return desired_motor_states;
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
	rollers<motorVelocityType> desired_motor_states = get_desired_motor_states(roller_states);
    rollers<motorVelocityType> motor_accelerations = get_wanted_motor_accels(desired_motor_states, dt);
    move_motor_accelerations(motor_accelerations);
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
	// double hue = optical.get_hue();
    double hue = 0.01;
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
