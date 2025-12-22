#ifndef MOTION_PROFILER_HPP
#define MOTION_PROFILER_HPP

/**
 * \file motion-profiler.hpp
 * @brief Motion profiling utilities for trajectory generation.
 *
 * Declares the `mp` class, which generates triangular and trapezoidal
 * velocity profiles that respect maximum acceleration and velocity
 * constraints. Used by the drivetrain to execute smooth linear and
 * angular motions.
 */

#include "api.h"
#include "structs.hpp"

/**
 * @brief Motion profiler for generating smooth velocity trajectories
 *
 * Generates velocity profiles for motion control that respect maximum
 * acceleration and velocity constraints. Supports both triangular (when
 * distance is short) and trapezoidal (when distance allows constant
 * velocity phase) profiles for optimal motion planning.
 */
class mp {
private:


public:
    /**
     * @brief Constructs a motion profiler with constraints
     *
     * Initializes the motion profiler with maximum acceleration and velocity
     * limits. These constraints determine the shape and duration of generated
     * motion profiles.
     *
     * @param[in] max_acceleration_ Maximum allowed acceleration in appropriate units
     * @param[in] max_velocity_ Maximum allowed velocity in appropriate units
     */
    mp(const double max_acceleration_,
       const double max_velocity_)
        : max_acceleration(max_acceleration_),
          max_velocity(max_velocity_)
    {}
    const double max_acceleration;
    const double max_velocity;
    double end_time;
    
    /**
     * @brief Checks if the motion profile has completed
     *
     * Determines whether the motion profile has finished executing by
     * comparing the current time to the calculated end time of the profile.
     *
     * @param[in] time Current time since profile start in seconds
     *
     * @return True if time >= end_time, false otherwise
     */
    bool profileFinished(double time);
    
    /**
     * @brief Gets the velocity at a given time for a specified distance
     *
     * Calculates the velocity at a specific time point in the motion profile
     * for a given total distance. Automatically selects between triangular
     * and trapezoidal profiles based on whether the distance allows reaching
     * maximum velocity. The end_time member is updated during calculation.
     *
     * @param[in] time Current time since profile start in seconds
     * @param[in] distance Total distance to travel in appropriate units
     *
     * @return Velocity at the specified time in appropriate units
     */
    double velocity(double time, double distance);
    
    /**
     * @brief Generates a triangular motion profile velocity
     *
     * Creates a triangular velocity profile that accelerates to a peak
     * velocity then decelerates to zero, without a constant velocity phase.
     * This is used when the distance is too short to reach maximum velocity.
     * The profile consists of two phases: acceleration and deceleration.
     *
     * @param[in] time Current time since profile start in seconds
     * @param[in] distance Total distance to travel in appropriate units
     *
     * @return Velocity at the specified time in appropriate units
     */
    double triangular_motion_profile(double time, double distance);
    
    /**
     * @brief Generates a trapezoidal motion profile velocity
     *
     * Creates a trapezoidal velocity profile with three phases: acceleration
     * to maximum velocity, constant velocity, and deceleration to zero.
     * This is used when the distance is sufficient to reach and maintain
     * maximum velocity for a period.
     *
     * @param[in] time Current time since profile start in seconds
     * @param[in] distance Total distance to travel in appropriate units
     *
     * @return Velocity at the specified time in appropriate units
     */
    double trapezoidal_motion_profile(double time, double distance);
};




#endif // MOTION_PROFILER_HPP
