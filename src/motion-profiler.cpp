#include "motion-profiler.hpp"

bool mp::profileFinished(double time) {
    return (time >= end_time);
}

double mp::velocity(double time, double distance) {
    const double t_1 = max_velocity / max_acceleration;
    const double c = max_velocity * t_1 / 2;
    const double b = distance - 2 * c;
    double velocity = 0.0;
    if (b < 0) {
        velocity = triangular_motion_profile(time, distance);
    }
    else {
        velocity = trapezoidal_motion_profile(time, distance);
    }
    return velocity;
}

double mp::triangular_motion_profile(double time, double distance) {
    const double h = sqrt(distance * max_acceleration);
    const double z = 2 * distance / h;
    end_time = z;
    if (time < z / 2) {
        return (max_acceleration * time);
    }
    else {
        return (-max_acceleration * (time - z/2) + max_acceleration * z/2);
    }
}

double mp::trapezoidal_motion_profile(double time, double distance) {
    const double h = sqrt(distance * max_acceleration);
    const double z = 2 * distance / h;
    const double u = 2 * max_velocity / max_acceleration;
    const double w = (distance - (1.0 / 2.0 * u * max_velocity)) / max_velocity;
    const double t_4 = u / 2;
    const double t_5 = t_4 + w;
    end_time = t_5 + t_4;
    if (time < t_4) {
        return (max_acceleration * time);    
    }
    else {
        if (time <= t_5) {
            return(max_velocity);
        }
        else {
            return (-max_acceleration * (time - t_5) + max_velocity);
        }
    }
}
