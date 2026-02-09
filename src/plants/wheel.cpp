// wheel_plant.cpp
#include "wheel.hpp"
#include <utility>

Wheel::Wheel(std::shared_ptr<MotorController> motor_) 
	: motor(std::move(motor_)) {}

void Wheel::set_linear_velocity(double rpm) {}
void Wheel::get_linear_velocity() {}
