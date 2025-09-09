#ifndef SYSTEM_IDENTIFICATION_HPP
#define SYSTEM_IDENTIFICATION_HPP

#include "api.h"
#include "structs.hpp"

void print_vector(const std::vector<input_output>& vec);

class compute {
private:
    pros::Motor& test_motor;
public:
    compute(pros::Motor& test_motor_) : test_motor(test_motor_) {}
    std::vector<input_output> fopdt_system_identification(int n);
};

#endif // SYSTEM_IDENTIFICATION_HPP
