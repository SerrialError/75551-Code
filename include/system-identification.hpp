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

class DCff {
private:
    const double K_a;
    const double K_v;
    const double K_s;
    static double sign(double x) {
        return (x > 0) - (x < 0);
    }
public:
    DCff(double K_a_, double K_v_, double K_s_) : K_a(K_a_), K_v(K_v_), K_s(K_s_) {}


    double compute_voltage(double alpha /*rad/s^2*/, double omega /*rad/s*/) const {
        double u = K_a * alpha + K_v * omega + K_s * sign(omega);
        return u;
    }
};

#endif // SYSTEM_IDENTIFICATION_HPP
