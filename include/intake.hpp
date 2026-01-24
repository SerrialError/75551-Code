#ifndef INTAKE_HPP
#define INTAKE_HPP

#include "api.h"

template<typename T>
struct rollers {
    T front;
    T back;
    auto asArray() {
        return std::array<std::reference_wrapper<T>, 2>{
            std::ref(front), std::ref(back)
        };
    }

    auto asArray() const {
        return std::array<std::reference_wrapper<const T>, 2>{
            std::cref(front), std::cref(back)
        };
    }

    T& operator[](size_t i) {
        switch (i) {
            case 0: return front;
            case 1: return back;
        }
    }

    const T& operator[](size_t i) const {
        switch (i) {
            case 0: return front;
            case 1: return back;
        }
    }
};

enum rollerStateType {
    intakeOff,   /**< Intake is off, both rollers stopped */
    intakeOnly,  /**< Only front roller running to intake blocks */
    bottomScore, /**< Front roller reversed to score at bottom height */
    midScore,    /**< Front roller forward, back roller reversed for mid height */
    topScore     /**< Both rollers forward for top height scoring */
};

struct motorVelocityType {
    float voltage;                    /**< Desired velocity in rad/s */
    pros::motor_brake_mode_e  brakeMode; /**< Brake mode (coast, brake, or hold) */
};

enum class SpecialState { hold, off };

using motorStateType = std::variant<float, SpecialState>;

inline motorStateType make_running(float s) { return s; }

inline motorStateType make_hold() { return SpecialState::hold; }

inline motorStateType make_off()  { return SpecialState::off; }

class intake {
private:
    pros::Controller master{pros::E_CONTROLLER_MASTER};
    rollers<std::reference_wrapper<pros::Motor>> motors;
    motorStateType motorState;

public:
    intake(const rollers<std::reference_wrapper<pros::Motor>>& motors_)
        : motors{ motors_.front, motors_.back}
    {}
    rollerStateType intakeState = intakeOff;

	void move_motor_states(const rollers<motorVelocityType>& motor_states);

	rollers<motorStateType> get_roller_states();

	motorVelocityType get_desired_motor_state(motorStateType wanted_roller_state, std::reference_wrapper<pros::Motor> motor);

	rollers<motorVelocityType> get_desired_motor_states(rollers<motorStateType> wanted_roller_states);
    
	void update_intake_state();
};

#endif // INTAKE_HPP