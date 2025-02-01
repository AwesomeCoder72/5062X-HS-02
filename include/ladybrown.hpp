#pragma once

#include "pros/motors.hpp"
#include "pros/rotation.hpp"

extern pros::Motor LadyBrownMech;
extern void liftControl();
extern pros::Rotation LadyBrownRotationSensor;

extern void nextState();

void spin_lady_brown_driver(int ladyBrownUpButtonValue, int ladyBrownDownButtonValue);