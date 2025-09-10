#ifndef MOTION_PROFILER_HPP
#define MOTION_PROFILER_HPP

#include "api.h"
#include "system-identification.hpp"
#include "ff-velocity-controller.hpp"
#include "structs.hpp"

void oned_motion_profiler(pros::Motor& test_motor, ff_constants test_motor_constants);

#endif // motion-profiler.hpp