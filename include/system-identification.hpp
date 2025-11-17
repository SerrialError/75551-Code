#ifndef SYSTEM_IDENTIFICATION_HPP
#define SYSTEM_IDENTIFICATION_HPP

#include "api.h"
#include "structs.hpp"
#include "motor-controller.hpp"

class print_data {
private:
    MotorController *motor;

public:
    print_data(MotorController *motor_) : motor(motor_) {}
    std::vector<input_output> motor_data;
    void print_vector();
    void update_motor_data();
};

#endif // SYSTEM_IDENTIFICATION_HPP