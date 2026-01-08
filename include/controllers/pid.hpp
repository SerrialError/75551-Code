#ifndef PID_HPP
#define PID_HPP
#include "controller.hpp"

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
    double compute(const double& setpoint, const double& output) override;
};

#endif // pid.hpp