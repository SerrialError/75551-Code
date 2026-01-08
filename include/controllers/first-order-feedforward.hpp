#ifndef FIRST_ORDER_FEEDFORWARD_HPP
#define FIRST_ORDER_FEEDFORWARD_HPP
#include "controller.hpp"

struct FirstOrderFeedforwardConstants {
    float Ka;         /**< Acceleration constant (V/(rad/s^2)) */
    float Kv;         /**< Velocity constant (V/(rad/s)) */
    float Ks;         /**< Static friction constant (V) */
    float max_ang_vel; /**< Maximum angular velocity (rad/s) */
    float max_voltage; /**< Maximum voltage (V) */ 
};

struct FirstOrderFeedforwardState { float acceleration, velocity; };

class FirstOrderFeedforward : Controller<FirstOrderFeedforwardConstants, FirstOrderFeedforwardState, float, float, ControllerType::Feedforward> {
public:
    using Base = Controller<FirstOrderFeedforwardConstants, FirstOrderFeedforwardState, float, float, ControllerType::Feedforward>;
    
	explicit FirstOrderFeedforward(FirstOrderFeedforwardConstants constants_)
        : Base(std::move(constants_)) {}

    // Override the concrete virtual function with the plant inversion feedforard
    float compute(const FirstOrderFeedforwardState& setpoint) override;
};

#endif // first-order-feedforward.hpp
