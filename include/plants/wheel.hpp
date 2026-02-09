#ifndef WHEEL_HPP
#define WHEEL_HPP

#include "plant.hpp"
#include "motor-controller.hpp"
#include <memory>

class Wheel : public Plant {
public:
    explicit Wheel(std::shared_ptr<MotorController> motor_);

    void set_linear_velocity(double rpm);
    void get_linear_velocity();

private:
	std::shared_ptr<MotorController> motor;
};

#endif // wheel.hpp
