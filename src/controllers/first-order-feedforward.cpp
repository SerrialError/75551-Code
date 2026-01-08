#include "controllers/first-order-feedforward.hpp"

double FirstOrderFeedforward::compute(const FirstOrderFeedforwardState& setpoint) {
    return constants.Ka * setpoint.acceleration + constants.Kv * setpoint.velocity + constants.Ks * sign(setpoint.velocity);
}