#ifndef FF_VELOCITY_CONTROLLER_HPP
#define FF_VELOCITY_CONTROLLER_HPP

#include "api.h"
#include "structs.hpp"
#include "helper-functions.hpp"

class DCff {
private:
    ff_constants constants;
public:
    DCff(ff_constants constants_) : constants(constants_) {}
    
    double compute_voltage(double alpha /*rad/s^2*/, double omega /*rad/s*/) const {
        double u = constants.K_a * alpha + constants.K_v * omega + constants.K_s * sign(omega);
        return u;
    }
};

class pd {
private:
    double r; //setpoint(desired velocity)
    double u; //control input
    double e; //error(aceleration)
    double y; //output(measured velocity)


public:

};
#endif // ff-velocity-controller.hpp
