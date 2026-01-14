/**
 * \file intake.cpp
 * @brief Implementation of the intake state machine and color sorting.
 *
 * Contains the logic that maps high-level intake states to motor commands,
 * computes desired velocities for each roller, and optionally adjusts the
 * state based on detected block color.
 */
#include "intake.hpp"
#include "structs.hpp"

void intake::move_motor_accelerations(const rollers<motorVelocityType>& motor_accelerations) {
    for (int i = 0; i < 2; i++) {
	    motors[i].move_acceleration(motor_accelerations[i]);
    }
}

rollers<motorVelocityType> intake::get_wanted_motor_accels(const rollers<motorVelocityType>& desired_motor_vels) {
    rollers<motorVelocityType> result{};
    for (size_t i = 0; i < 2; ++i) {
	    result[i].velocity = motors[i].get_desired_acceleration(desired_motor_vels[i].velocity);
		result[i].brakeMode = desired_motor_vels[i].brakeMode;
    }
    return result;
}

rollers<motorStateType> intake::get_roller_states(void) {
    rollers<motorStateType> result;
    // Map the high-level intake state to a desired state per roller.
    switch (intakeState) {
        case intakeOff:
            result = { make_off(), make_off() };
            break;

        case intakeOnly:
            result = { make_running(1.0), make_off() };
            break;

        case bottomScore:
            result = { make_running(-1.0), make_off() };
            break;

        case midScore:
            result = { make_running(0.8), make_running(-1.0) };
            break;

        case topScore:
            result = { make_running(1.0), make_running(1.0) };
            break;

        default:
            result = { make_off(), make_off() };
            break;
    }
    return result;
}

motorVelocityType intake::get_desired_motor_state(motorStateType wanted_roller_state, MotorController motor) {
    float wanted_velocity = 0.0;
    pros::motor_brake_mode_e wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;

    if (std::holds_alternative<float>(wanted_roller_state)) {
        float scale = std::get<float>(wanted_roller_state);     // e.g. 0.8
        wanted_velocity = motor.ff_constants.max_ang_vel * scale;
        wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
    } else {
        // it's a SpecialState
        switch (std::get<SpecialState>(wanted_roller_state)) {
            case SpecialState::off:
                wanted_velocity = 0.0;
                wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
                break;
            case SpecialState::hold:
                wanted_velocity = 0.0;
                wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD;
                break;
        }
    }

    wanted_velocity = motor.bound_velocity_to_deadband(wanted_velocity);
    return motorVelocityType{wanted_velocity, wanted_brake};
}

rollers<motorVelocityType> intake::get_desired_motor_states(rollers<motorStateType> wanted_roller_states) {
    rollers<motorVelocityType> desired_motor_states;
	for (size_t i = 0; i < 2; ++i) {
        desired_motor_states[i] = get_desired_motor_state(wanted_roller_states[i], motors[i]);
    }
	return desired_motor_states;
}

void intake::get_intake_state() {
    if (intakeState != intakeOff) {
        // If the block color is wrong, bump the state to route the piece differently.
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

void intake::update_intake_state(const float& dt) {
	if (colorSorting) {
        get_intake_state();
    }
    rollers<motorStateType> roller_states = get_roller_states();
	rollers<motorVelocityType> desired_motor_states = get_desired_motor_states(roller_states);
    rollers<motorVelocityType> motor_accelerations = get_wanted_motor_accels(desired_motor_states);
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
	// float hue = optical.get_hue();
    float hue = 0.01;
    master.print(0, 0, "%lf", hue);
    master.clear();
	float redAngularDistance = angularDistance(hue, redHue);
	float blueAngularDistance = angularDistance(hue, blueHue);
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
