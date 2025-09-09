#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"

class drivetrain {
private:
    pros::Motor& m1;
    pros::Motor& m2;
    pros::Motor& o1;
    pros::Motor& o2;
    pros::Motor& m3;
    pros::Motor& m4;
public:
    drivetrain(pros::Motor& m1_, pros::Motor& m2_, pros::Motor& o1_, pros::Motor& o2_, pros::Motor& m3_, pros::Motor& m4_) : m1(m1_), m2(m2_), o1(o1_), o2(o2_), m3(m3_), m4(m4_) {}

};

#endif // DRIVETRAIN_HPP
