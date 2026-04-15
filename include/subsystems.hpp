#pragma once

#include "EZ-Template/api.hpp"
#include "intake.hpp"
#include "api.h"

extern Drive chassis;

inline pros::adi::Pneumatics match_loader ('h', false, false);
inline pros::adi::Pneumatics wings ('f', false, false);
inline pros::adi::Pneumatics two_state_system ('b', false, false);
inline pros::Motor front(-4, pros::v5::MotorGears::blue);
inline pros::Motor back(-14, pros::v5::MotorGears::blue);

inline const rollers<std::reference_wrapper<pros::Motor>> intakeMotors{
	std::ref(front),
	std::ref(back)
};

inline intake Intake(intakeMotors);
