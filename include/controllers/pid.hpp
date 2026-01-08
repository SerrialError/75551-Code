#ifndef PID_HPP
#define PID_HPP
#include "controller.hpp"

struct PIDConstants { float Kp, Ki, Kd; };

class PID : public Controller<PIDConstants, float, float, float, ControllerType::Feedback> {
private:
    float timestep;
    float previous_error = 0.0;
    float error_integral = 0.0;

public:
    using Base = Controller<PIDConstants, float, float, float, ControllerType::Feedback>;

    explicit PID(PIDConstants constants_, float timestep_)
        : Base(std::move(constants_)), timestep(timestep_) {}

    // Override the concrete virtual function
    float compute(const float& setpoint, const float& output) override;
};

#endif // pid.hpp
