#ifndef MOTION_PROFILER_HPP
#define MOTION_PROFILER_HPP

#include "api.h"
#include "structs.hpp"

class mp {
private:


public:
    mp(const double max_acceleration_,
       const double max_velocity_)
        : max_acceleration(max_acceleration_),
          max_velocity(max_velocity_)
    {}
    const double max_acceleration;
    const double max_velocity;
    double end_time;
    bool profileFinished(double time);
    double velocity(double time, double distance);
    double triangular_motion_profile(double time, double distance);
    double trapezoidal_motion_profile(double time, double distance);
};




#endif // MOTION_PROFILER_HPP
