#ifndef INTAKE_HPP
#define INTAKE_HPP

#include "api.h"
#include "structs.hpp"
#include "ff-velocity-controller.hpp"

class intake {
private:
    pros::Controller master{pros::E_CONTROLLER_MASTER};
    const rollers<std::reference_wrapper<pros::Motor>> motors;
    pros::Optical& optical;
    const rollers<ff_constants> motor_constants;
    const rollers<DCff> motor_ffs;
    motorStateType motorState;

public:
    intake(const rollers<std::reference_wrapper<pros::Motor>>& motors_,
              pros::Optical& optical_,
              const rollers<ff_constants>& motor_constants_)
        : motors(motors_),
          optical(optical_),
          motor_constants(motor_constants_),
          motor_ffs{ DCff(motor_constants_.fb),
                     DCff(motor_constants_.ft),
                     DCff(motor_constants_.bb),
                     DCff(motor_constants_.bt) }
    {}
    rollerStateType intakeState;
    double sgn(double x) {
        if (x > 0) {
            return 1.0;
        } else if (x < 0) {
            return -1.0;
        } else {
            return 0.0;
        }
    }
    rollers<double> get_motor_max_accel(void);

    void move_wheel_accels(const rollers<double>& wheel_accelerations);

    void move_wheel_volts(const rollers<double>& wheel_voltages);
    
    rollers<wheel_vel_lim> get_motor_vel_limits(const double& dt);
    
    double get_motor_max_accel(pros::Motor motor, const ff_constants motor_constants_);

    double get_wanted_motor_vel(pros::Motor motor, const ff_constants motor_constants_, motorStateType wanted_roller_state, const double& dt);
    
    rollers<motorStateType> get_roller_states(void);
    
    rollers<double> get_wanted_motor_accels(const rollers<double>& wanted_motor_vels, const double& dt);
    
    void update_intake_state(const double& dt);
};

#endif // INTAKE_HPP
