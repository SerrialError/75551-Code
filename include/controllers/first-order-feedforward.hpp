#ifndef FIRST_ORDER_FEEDFORWARD_HPP
#define FIRST_ORDER_FEEDFORWARD_HPP
#include "controller.hpp"

struct FirstOrderFeedforwardConstants {
    double Ka;         /**< Acceleration constant (V/(rad/s^2)) */
    double Kv;         /**< Velocity constant (V/(rad/s)) */
    double Ks;         /**< Static friction constant (V) */
    double max_ang_vel; /**< Maximum angular velocity (rad/s) */
    double max_voltage; /**< Maximum voltage (V) */ 
};

struct FirstOrderFeedforwardState { double acceleration, velocity; };

class FirstOrderFeedforward : Controller<FirstOrderFeedforwardConstants, FirstOrderFeedforwardState, double, double, ControllerType::Feedforward> {
public:
    using Base = Controller<FirstOrderFeedforwardConstants, FirstOrderFeedforwardState, double, double, ControllerType::Feedforward>;
    
	explicit FirstOrderFeedforward(FirstOrderFeedforwardConstants constants_)
        : Base(std::move(constants_)) {}

    // Override the concrete virtual function with the plant inversion feedforard
    double compute(const FirstOrderFeedforwardState& setpoint) override;
};

#endif // first-order-feedforward.hpp