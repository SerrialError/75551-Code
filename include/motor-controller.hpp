#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include "api.h"
#include "structs.hpp"
#include "ff-velocity-controller.hpp"
#include "helper-functions.hpp"

class MotorController {
private:
    const pros::Motor motor;
    const ff_constants motor_constants;
    const DCff motor_ff;

public:
    MotorController(const std::reference_wrapper<pros::Motor>& motor_,
              const ff_constants& motor_constants_)
        : motor(motor_),
          motor_constants(motor_constants_),
          motor_ff(motor_constants_)
    {}
    
	void move_motor_acceleration(const double& desired_acceleration);

	double get_motor_max_accel(const int& direction);

	void move_motor_voltage(const double& voltage);

	void move_motor_volts_time(const double& voltage, const int& time);

	wheel_vel_bounds get_motor_vel_bounds(const double& dt);

	double get_desired_motor_acceleration(const double& desired_motor_vels, const double& dt);

	double bound_desired_motor_velocity(const double& desired_velocity, const double& dt);
};

#endif // MOTOR_CONTROLLER_HPP
