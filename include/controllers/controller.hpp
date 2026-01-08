#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

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

#endif // controller.hpp