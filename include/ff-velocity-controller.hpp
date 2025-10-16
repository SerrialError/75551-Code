#ifndef FF_VELOCITY_CONTROLLER_HPP
#define FF_VELOCITY_CONTROLLER_HPP

#include "api.h"
#include "structs.hpp"

class DCff {
private:
    ff_constants constants;
    static double sign(double x) {
        return (x > 0) - (x < 0);
    }
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

    double e = r - y;
        // error is the difference of desired velocity and measured velocity
    
public:

}
#endif // ff-velocity-controller.hpp