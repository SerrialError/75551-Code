#ifndef INTAKE_HPP
#define INTAKE_HPP

#include "api.h"
#include "structs.hpp"
#include "ff-velocity-controller.hpp"
#include "motor-controller.hpp"
#include "helper-functions.hpp"

class intake {
private:
    pros::Controller master{pros::E_CONTROLLER_MASTER};
    rollers<MotorController> motors;
    motorStateType motorState;
	const double redHue = 4.0;
	const double blueHue = 200.0;
	const double redHueUncertainty = 7.0;
    const double blueHueUncertainty = 20.0;

public:
    intake(const rollers<std::reference_wrapper<pros::Motor>>& motors_,
              const rollers<ff_constants>& motor_constants_)
        : motors{ MotorController(motors_.front, motor_constants_.front, "front"),
                  MotorController(motors_.back, motor_constants_.back, "back") }
    {}
    color allianceColor = red;
    rollerStateType intakeState = intakeOff;
	bool colorSorting = false;
	void move_motor_accelerations(const rollers<motorVelocityType>& motor_accelerations);
	rollers<motorVelocityType> get_wanted_motor_accels(const rollers<motorVelocityType>& desired_motor_vels, const double& dt);

	rollers<motorStateType> get_roller_states(void);

	motorVelocityType get_desired_motor_state(motorStateType wanted_roller_state, MotorController motor);

	rollers<motorVelocityType> get_desired_motor_states(rollers<motorStateType> wanted_roller_states);

	void get_intake_state();

	void update_intake_state(const double& dt);

	bool is_correct_color(void);

	color get_block_color(void);
	
	double angularDistance(double a, double b) {
    	double d = fabs(a - b);
    	if (d > 180.0) d = 360.0 - d;
    	return d;
	}
};

#endif // INTAKE_HPP
