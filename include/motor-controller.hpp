#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include "api.h"
#include "structs.hpp"
#include "ff-velocity-controller.hpp"
#include "helper-functions.hpp"

class MotorController {
private:
    const DCff motor_ff;
    std::vector<input_output> motor_data;

public:
    MotorController(const std::reference_wrapper<pros::Motor>& motor_,
              const ff_constants& motor_constants_,
              std::string_view motor_name_)
        : motor(motor_),
          motor_constants(motor_constants_),
          motor_ff(motor_constants_),
          motor_name(motor_name_)
    {}
    const ff_constants motor_constants;
    
	void print_vector();
    
    void update_motor_data();
    
    const std::reference_wrapper<pros::Motor> motor;
    
	std::string_view motor_name;

	void move_motor_acceleration(const motorVelocityType& desired_acceleration);

	double bound_velocity_to_deadband(double desired_velocity);

	double get_motor_max_accel(bool reverse, int direction);

	void move_motor_voltage(const double& voltage);

	void move_motor_volts_time(const double& voltage, const int& time);

	wheel_vel_bounds get_motor_vel_bounds(const double& dt);

	double get_desired_motor_acceleration(const double& desired_motor_vels, const double& dt);

	double bound_desired_motor_velocity(const double& desired_velocity, const double& dt);
};

#endif // MOTOR_CONTROLLER_HPP
