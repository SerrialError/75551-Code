#ifndef FF_VELOCITY_CONTROLLER_HPP
#define FF_VELOCITY_CONTROLLER_HPP

/**
 * \file ff-velocity-controller.hpp
 * @brief Feedforward velocity controller interfaces.
 *
 * Declares the `DCff` feedforward controller class for DC motors and a
 * placeholder `pd` controller. These classes are used to compute motor
 * voltages from desired accelerations and velocities using identified
 * motor constants.
 */

#include "api.h"
#include "structs.hpp"
#include "helper-functions.hpp"


enum class ControllerType { Feedback, Feedforward };

// Forward declaration of primary template
template <
    typename Constants,
    typename State,
    typename Output,
    typename Input,
    ControllerType controller>
class Controller;

template <typename Constants, typename State, typename Output, typename Input>
class Controller<Constants, State, Output, Input, ControllerType::Feedback> {
protected:
    Constants constants;
public:
    explicit Controller(Constants constants_) : constants(std::move(constants_)) {}
    virtual ~Controller() = default;

    virtual Input compute(const State& setpoint, const Output& output) = 0;
};

template <typename Constants, typename State, typename Output, typename Input>
class Controller<Constants, State, Output, Input, ControllerType::Feedforward> {
protected:
    Constants constants;
public:
    explicit Controller(Constants constants_) : constants(std::move(constants_)) {}
    virtual ~Controller() = default;

    virtual Input compute(const State& setpoint) = 0;
};

struct PIDConstants { double Kp, Ki, Kd; };

class PID : public Controller<PIDConstants, double, double, double, ControllerType::Feedback> {
private:
    double timestep;
    double previous_error = 0.0;
    double error_integral = 0.0;

public:
    using Base = Controller<PIDConstants, double, double, double, ControllerType::Feedback>;

    explicit PID(PIDConstants constants_, double timestep_)
        : Base(std::move(constants_)), timestep(timestep_) {}

    // Override the concrete virtual function
    double compute(const double& setpoint, const double& output) override {
        double error = setpoint - output;
        error_integral += error * timestep;
        double error_derivative = (error - previous_error) / timestep;
        previous_error = error;
        return constants.Kp * error + constants.Ki * error_integral + constants.Kd * error_derivative;
    }
};

/**
 * @brief DC motor feedforward controller
 *
 * Implements a feedforward control law for DC motors based on the motor's
 * physical characteristics. The controller calculates the voltage needed to
 * achieve a desired acceleration and velocity, accounting for back-EMF,
 * acceleration torque, and static friction.
 */

struct FirstOrderFeedforwardConstants { double Kv, Ka, Ks; };

struct FirstOrderFeedforwardState { double acceleration, velocity };

class FirstOrderFeedforward : Controller<FirstOrderFeedforwardConstants, FirstOrderFeedforwardState, double, double, ControllerType::Feedforward> {
public:
    using Base = Controller<FirstOrderFeedforwardConstants, FirstOrderFeedforwardState, double, double, ControllerType::Feedforward>;
    
	explicit FirstOrderFeedforward(FirstOrderFeedforwardConstants constants_)
        : Base(std::move(constants_)) {}

    // Override the concrete virtual function with the plant inversion feedforard
    double compute(const FirstOrderFeedforwardState& setpoint) override {
        return constants.Ka * setpoint.acceleration + constants.Kv * setpoint.velocity + constants.Ks * sign(setpoint.velocity);
    }
};

#endif // ff-velocity-controller.hpp
