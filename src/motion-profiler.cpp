#include "motion-profiler.hpp"

bool mp::profileFinished(double time) {
    return (time >= end_time);
}

double mp::velocity(double time, double distance) {
    
    const double t_1 = max_velocity / max_acceleration;
    const double c = max_velocity * t_1 / 2;
    const double b = distance - 2 * c;
    const double t_2 = b / max_velocity + t_1;
    const double theta = atan(max_acceleration);
    const double alpha = M_PI - 2 * theta;
    const double l = distance / sin(alpha);
    const double t_3 = l * cos(theta);
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
    const double t_1 = max_velocity / max_acceleration;
    const double c = max_velocity * t_1 / 2;
    const double b = distance - 2 * c;
    const double t_2 = b / max_velocity + t_1;
    const double theta = atan(max_acceleration);
    const double alpha = M_PI - 2 * theta;
    const double l = distance / sin(alpha);
    const double t_3 = l * cos(theta);
    end_time = 2 * t_3;
    if (time < l * cos(theta)) {
        return (max_acceleration * time);
    }
    else {
        return (-max_acceleration * (time - t_3) + max_acceleration * t_3);
    }
}

double mp::trapezoidal_motion_profile(double time, double distance) {
    const double t_1 = max_velocity / max_acceleration;
    const double c = max_velocity * t_1 / 2;
    const double b = distance - 2 * c;
    const double t_2 = b / max_velocity + t_1;
    const double theta = atan(max_acceleration);
    const double alpha = M_PI - 2 * theta;
    const double l = distance / sin(alpha);
    const double t_3 = l * cos(theta);
    end_time = (max_acceleration * t_2 + max_velocity)/max_acceleration;
    if (time < t_1) {
        return (max_acceleration * time);    
    }
    else {
        if (time <= t_2) {
            return(max_velocity);
        }
        else {
            return (-max_acceleration * (time - t_2) + max_velocity);
        }
    }
}