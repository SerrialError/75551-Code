#ifndef PID_HPP
#define PID_HPP
#include "controller.hpp"

enum class PIDState { velocity, position };

struct PIDConstants { 
	float Kp, Ki, Kd, max_integral, integral_start; 
	PIDState state; 
};

class PID : public Controller<PIDConstants, float, float, float, ControllerType::Feedback> {
private:
    const float timestep;
    float previous_output = 0.f;
	float previous_error = 0.f;
    float error_integral = 0.f;

public:
    using Base = Controller<PIDConstants, float, float, float, ControllerType::Feedback>;

    explicit PID(PIDConstants constants_, const float timestep_)
        : Base(std::move(constants_)), timestep(timestep_) {}

    // Override the concrete virtual function
    float compute(const float& setpoint, const float& output) override;

	void reset(const float setpoint, const float output);
};

#endif // pid.hpp
