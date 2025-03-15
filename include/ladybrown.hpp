#pragma once

#include "pros/motors.hpp"
#include "pros/rotation.hpp"

extern pros::Motor LadyBrownMech;
extern void liftControl();
extern pros::Rotation LadyBrownRotationSensor;

extern bool toggleThrottleTargetSpeed;

extern void nextState(int theNextState = -1);

extern int currState;

void spin_lady_brown_driver(int ladyBrownUpButtonValue, int ladyBrownDownButtonValue);