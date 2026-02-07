#ifndef WHEEL_HPP
#define WHEEL_HPP

#include "plant.hpp"
#include "motor-controller.hpp"

class Wheel : public Plant {
public:
    explicit Wheel(MotorController motor);

    void set_linear_velocity(double rpm);
    void get_linear_velocity();

    void update() override;

private:
	MotorController motor;
};

#endif // wheel.hpp
