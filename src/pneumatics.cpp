#include "pneumatics.hpp"
#include "intake.cpp"

void upperPiston() {
    if (rollerStateType::intakeOnly) {
        upperPistonState::retracted;
    } else if (rollerStateType::topScore) {
        upperPistonState::extended;
    } else {
        upperPistonState::retracted;
    }
}

bool lowerPiston() {
    if (matchLoad::f) {
        lowerPistonState::retracted;
        return false;
    } else if (matchLoad::t) {
        lowerPistonState::extended;
        return true;
    } else {
        lowerPistonState::retracted;
        return false;
    }
}

