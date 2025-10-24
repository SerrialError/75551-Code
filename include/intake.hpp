#ifndef INTAKE_HPP
#define INTAKE_HPP

#include "api.h"
#include "structs.hpp"
#include "ff-velocity-controller.hpp"

class intake {
private:
    pros::Controller master{pros::E_CONTROLLER_MASTER};
    const rollers<std::reference_wrapper<pros::Motor>> motors;
    const rollers<ff_constants> motor_constants;
    const rollers<DCff> motor_ffs;
    motorStateType motorState;
    const color allianceColor;
	const double redHue = 4.0;
	const double blueHue = 200.0;
	const double redHueUncertainty = 7.0;
    const double blueHueUncertainty = 20.0;

public:
    intake(const rollers<std::reference_wrapper<pros::Motor>>& motors_,
              pros::Optical& optical_,
              const rollers<ff_constants>& motor_constants_,
              color allianceColor_)
        : motors(motors_),
          optical(optical_),
          motor_constants(motor_constants_),
          motor_ffs{ DCff(motor_constants_.fb),
                     DCff(motor_constants_.ft),
                     DCff(motor_constants_.bb),
                     DCff(motor_constants_.bt) },
          allianceColor(allianceColor_)
    {}
    pros::Optical& optical;
    rollerStateType intakeState = intakeOff;
	bool colorSorting = true;
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
    
    double get_motor_max_accel(pros::Motor& motor, const ff_constants motor_constants_, int direction);

    double get_wanted_motor_vel(pros::Motor& motor, const ff_constants motor_constants_, motorStateType wanted_roller_state, const double& dt);
    
    rollers<motorStateType> get_roller_states(void);
    
    motorVelocityType get_wanted_motor_accel(pros::Motor& motor, const motorVelocityType& wanted_motor_vels, const double& dt);
    
    bool is_correct_color(void);

	color get_block_color(void);

    void update_intake_state(const double& dt);
	
	double angularDistance(double a, double b) {
    	double d = fabs(a - b);
    	if (d > 180.0) d = 360.0 - d;
    	return d;
	}
	void get_intake_state(void);
};

#endif // INTAKE_HPP
