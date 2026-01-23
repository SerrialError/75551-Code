#ifndef INTAKE_HPP
#define INTAKE_HPP

// #include "api.h"
/*
class intake {
private:
    pros::Controller master{pros::E_CONTROLLER_MASTER};
    rollers<MotorController> motors;
    motorStateType motorState;

public:
    intake(const rollers<std::reference_wrapper<pros::Motor>>& motors_,
           const rollers<FirstOrderFeedforwardConstants>& motor_ff_constants_,
		   const rollers<PIDConstants>& motor_pid_constants_,
		   const float timestep)
        : motors{ MotorController(motors_.front, motor_ff_constants_.front, motor_pid_constants_.front, timestep, "front"),
                  MotorController(motors_.back, motor_ff_constants_.back, motor_pid_constants_.front, timestep, "back") }
    {}
    color allianceColor = red;
    rollerStateType intakeState = intakeOff;
	bool colorSorting = false;

	void move_motor_accelerations(const rollers<motorVelocityType>& motor_accelerations);
	
	rollers<motorVelocityType> get_wanted_motor_accels(const rollers<motorVelocityType>& desired_motor_vels);

	rollers<motorStateType> get_roller_states(void);

	motorVelocityType get_desired_motor_state(motorStateType wanted_roller_state, MotorController motor);

	rollers<motorVelocityType> get_desired_motor_states(rollers<motorStateType> wanted_roller_states);

	void get_intake_state();

	void update_intake_state(const float& dt);
};
*/
#endif // INTAKE_HPP