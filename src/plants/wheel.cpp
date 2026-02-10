#include "wheel.hpp"
#include "structs.hpp"

Wheel::Wheel(std::shared_ptr<MotorController> motor_, double radius_) 
	: motor(std::move(motor_)), radius(radius_) {}
    
void Wheel::move_linear_acceleration(motorVelocityType& linear_acceleration) {
	motorVelocityType angular_acceleration;
	angular_acceleration.velocity = linear_acceleration.velocity / radius;
	angular_acceleration.brakeMode = linear_acceleration.brakeMode;
	motor->move_acceleration(angular_acceleration);
}
double Wheel::get_linear_velocity() {
	return motor->get_angular_velocity() * radius;	
}
