#pragma once

#include "EZ-Template/api.hpp"
#include "intake.hpp"
#include "api.h"

extern Drive chassis;

inline pros::adi::Pneumatics match_loader ('a', false, false);
inline pros::adi::Pneumatics wings ('b', false, false);
inline pros::Motor front(-2, pros::v5::MotorGears::blue);
inline pros::Motor back(-9, pros::v5::MotorGears::blue);

inline const rollers<std::reference_wrapper<pros::Motor>> intakeMotors{
	std::ref(front),
	std::ref(back)
};

inline intake Intake(intakeMotors);

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
