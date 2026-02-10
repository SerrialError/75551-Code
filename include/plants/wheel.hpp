#ifndef WHEEL_HPP
#define WHEEL_HPP

#include "plant.hpp"
#include "motor-controller.hpp"
#include "structs.hpp"
#include <memory>

class Wheel : public Plant {
public:
    explicit Wheel(std::shared_ptr<MotorController> motor_, double radius_);

    void move_linear_acceleration(motorVelocityType& linear_acceleration);
    double get_linear_velocity();

private:
	const double radius;
	std::shared_ptr<MotorController> motor;
};

#endif // wheel.hpp
