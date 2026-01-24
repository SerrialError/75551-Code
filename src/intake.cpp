#include "intake.hpp"

void intake::move_motor_states(const rollers<motorVelocityType>& motor_states) {
    for (int i = 0; i < 2; i++) {
	    motors[i].get().move_voltage(static_cast<long>(motor_states[i].voltage * 1000.f));
    }
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

motorVelocityType intake::get_desired_motor_state(motorStateType wanted_roller_state, std::reference_wrapper<pros::Motor> motor) {
    float wanted_voltage = 0.0;
    pros::motor_brake_mode_e wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;

    if (std::holds_alternative<float>(wanted_roller_state)) {
        float scale = std::get<float>(wanted_roller_state);     // e.g. 0.8
        wanted_voltage = 12.f * scale;
        wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
    } else {
        // it's a SpecialState
        switch (std::get<SpecialState>(wanted_roller_state)) {
            case SpecialState::off:
                wanted_voltage = 0.f;
                wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST;
                break;
            case SpecialState::hold:
                wanted_voltage = 0.f;
                wanted_brake = pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD;
                break;
        }
    }

    return motorVelocityType{wanted_voltage, wanted_brake};
}

rollers<motorVelocityType> intake::get_desired_motor_states(rollers<motorStateType> wanted_roller_states) {
    rollers<motorVelocityType> desired_motor_states;
	for (size_t i = 0; i < 2; ++i) {
        desired_motor_states[i] = get_desired_motor_state(wanted_roller_states[i], motors[i]);
    }
	return desired_motor_states;
}

void intake::update_intake_state() {
    rollers<motorStateType> roller_states = get_roller_states();
	rollers<motorVelocityType> desired_motor_states = get_desired_motor_states(roller_states);
    move_motor_states(desired_motor_states);
}