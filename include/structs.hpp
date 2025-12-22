// structs.hpp
#ifndef STRUCTS_HPP
#define STRUCTS_HPP

/**
 * \file structs.hpp
 * @brief Common strongly-typed data structures for the robot.
 *
 * Declares generic containers for wheels and rollers, pose and motion types,
 * feedforward constants, velocity bounds, and various enums used throughout
 * the drivetrain, intake, and localization code.
 */

#include "api.h"

/**
 * @brief Template structure for six-wheel holonomic drivetrain
 *
 * Represents the six wheels of a holonomic drivetrain: four mecanum wheels
 * (m1, m2, m3, m4) and two omni wheels (o1, o2). Provides array-like access
 * and iteration capabilities for uniform processing of all wheels.
 *
 * @tparam T Type of data stored for each wheel (e.g., MotorController, double, etc.)
 */
template<typename T>
struct wheels {
    T m1, m2, o1, o2, m3, m4;

    // non-const view as array of reference_wrapper
    auto asArray() {
        return std::array<std::reference_wrapper<T>, 6>{
            std::ref(m1), std::ref(m2), std::ref(o1),
            std::ref(o2), std::ref(m3), std::ref(m4)
        };
    }

    // const-view: reference to const T
    auto asArray() const {
        return std::array<std::reference_wrapper<const T>, 6>{
            std::cref(m1), std::cref(m2), std::cref(o1),
            std::cref(o2), std::cref(m3), std::cref(m4)
        };
    }

    // convenient index access (non-const)
    T& operator[](size_t i) {
        switch (i) {
            case 0: return m1;
            case 1: return m2;
            case 2: return o1;
            case 3: return o2;
            case 4: return m3;
            default: return m4;
        }
    }

    // index access const
    const T& operator[](size_t i) const {
        switch (i) {
            case 0: return m1;
            case 1: return m2;
            case 2: return o1;
            case 3: return o2;
            case 4: return m3;
            default: return m4;
        }
    }
};

/**
 * @brief Template structure for two-roller intake system
 *
 * Represents the front and back rollers of an intake system. Provides
 * array-like access and iteration capabilities for uniform processing
 * of both rollers.
 *
 * @tparam T Type of data stored for each roller (e.g., MotorController, double, etc.)
 */
template<typename T>
struct rollers {
    T front;
    T back;
    /**
     * @brief Returns a non-const array view of both rollers
     *
     * Creates an array of reference wrappers to both rollers, allowing
     * iteration and uniform processing.
     *
     * @return Array of reference wrappers to front and back rollers
     */
    auto asArray() {
        return std::array<std::reference_wrapper<T>, 2>{
            std::ref(front), std::ref(back)
        };
    }

    /**
     * @brief Returns a const array view of both rollers
     *
     * Creates an array of const reference wrappers to both rollers for
     * read-only access.
     *
     * @return Array of const reference wrappers to front and back rollers
     */
    auto asArray() const {
        return std::array<std::reference_wrapper<const T>, 2>{
            std::cref(front), std::cref(back)
        };
    }

    /**
     * @brief Index access operator for non-const access
     *
     * Provides array-like indexing: 0=front, 1=back.
     *
     * @param[in] i Index of roller (0 or 1)
     *
     * @return Reference to the roller at index i
     */
    T& operator[](size_t i) {
        switch (i) {
            case 0: return front;
            case 1: return back;
        }
    }

    /**
     * @brief Index access operator for const access
     *
     * Provides array-like indexing for read-only access: 0=front, 1=back.
     *
     * @param[in] i Index of roller (0 or 1)
     *
     * @return Const reference to the roller at index i
     */
    const T& operator[](size_t i) const {
        switch (i) {
            case 0: return front;
            case 1: return back;
        }
    }
};

/**
 * @brief Robot pose in 2D space
 *
 * Represents the position and orientation of the robot on the field.
 * Coordinates are typically in meters, and theta is in radians.
 */
struct pose {
    double x;      /**< X coordinate in meters */
    double y;      /**< Y coordinate in meters */
    double theta;  /**< Orientation angle in radians */
};

/**
 * @brief Feedforward motor constants
 *
 * Contains the constants used in the feedforward motor model:
 * u = K_a * alpha + K_v * omega + K_s * sign(omega)
 * where u is voltage, alpha is acceleration, and omega is velocity.
 */
struct ff_constants {
    double K_a;         /**< Acceleration constant (V/(rad/s^2)) */
    double K_v;         /**< Velocity constant (V/(rad/s)) */
    double K_s;         /**< Static friction constant (V) */
    double max_ang_vel; /**< Maximum angular velocity (rad/s) */
    double max_voltage; /**< Maximum voltage (V) */
};

/**
 * @brief Velocity bounds for a wheel
 *
 * Represents the minimum and maximum achievable velocities for a wheel
 * at a given point in time, accounting for acceleration constraints.
 */
struct wheel_vel_bounds {
    double min; /**< Minimum achievable velocity */
    double max; /**< Maximum achievable velocity */
};

/**
 * @brief Input-output data pair for system identification
 *
 * Stores a voltage input and corresponding velocity output measurement
 * for use in system identification and motor characterization.
 */
struct input_output {
    double u; /**< Input voltage (V) */
    double x; /**< Output velocity (rad/s) */
};

/**
 * @brief Intake system operating states
 *
 * Defines the possible states for the intake system, each corresponding
 * to different motor velocity configurations for the rollers.
 */
enum rollerStateType {
    intakeOff,   /**< Intake is off, both rollers stopped */
    intakeOnly,  /**< Only front roller running to intake blocks */
    bottomScore, /**< Front roller reversed to score at bottom height */
    midScore,    /**< Front roller forward, back roller reversed for mid height */
    topScore     /**< Both rollers forward for top height scoring */
};

/**
 * @brief Special motor states beyond velocity control
 *
 * Defines special states that don't correspond to a velocity but rather
 * to a specific motor behavior (hold or off).
 */
enum class SpecialState { hold, off };

/**
 * @brief Motor state type that can be a velocity or special state
 *
 * A variant type that can hold either a double (velocity scale factor)
 * or a SpecialState (hold or off). This allows flexible motor control.
 */
using motorStateType = std::variant<double, SpecialState>;

/**
 * @brief Creates a running motor state with velocity scale
 *
 * Creates a motor state representing a running motor at a specified
 * velocity scale factor (typically 0.0 to 1.0).
 *
 * @param[in] s Velocity scale factor (0.0 to 1.0)
 *
 * @return Motor state with the specified velocity scale
 */
inline motorStateType make_running(double s) { return s; }

/**
 * @brief Creates a hold motor state
 *
 * Creates a motor state that holds the motor position using brake.
 *
 * @return Motor state set to hold
 */
inline motorStateType make_hold() { return SpecialState::hold; }

/**
 * @brief Creates an off motor state
 *
 * Creates a motor state that turns the motor off (coast).
 *
 * @return Motor state set to off
 */
inline motorStateType make_off()  { return SpecialState::off; }

/**
 * @brief Block color enumeration
 *
 * Represents the possible colors of game blocks that can be detected
 * by the intake's color sensor.
 */
enum color {
    red,  /**< Red alliance block */
    blue, /**< Blue alliance block */
	none  /**< No block detected or uncertain color */
};

/**
 * @brief Motor velocity command with brake mode
 *
 * Combines a desired velocity with a brake mode setting for motor control.
 * The brake mode determines behavior when velocity is zero.
 */
struct motorVelocityType {
    double velocity;                    /**< Desired velocity in rad/s */
    pros::motor_brake_mode_e  brakeMode; /**< Brake mode (coast, brake, or hold) */
};

/**
 * @brief Differential drive velocity commands
 *
 * Represents linear and angular velocity commands for a differential
 * drive robot (or holonomic robot using differential drive control).
 */
struct differentialVels {
    double linear;  /**< Linear velocity in m/s (forward/backward) */
    double angular; /**< Angular velocity in rad/s (rotation) */
};

/**
 * @brief Autonomous routine identifiers
 *
 * Enumeration of the different autonomous routines available,
 * identified by alliance color and starting position.
 */
enum autons {
    blueRight, /**< Blue alliance, right starting position */
    blueLeft,  /**< Blue alliance, left starting position */
    redRight,  /**< Red alliance, right starting position */
    redLeft    /**< Red alliance, left starting position */
};

#endif // STRUCTS_HPP
