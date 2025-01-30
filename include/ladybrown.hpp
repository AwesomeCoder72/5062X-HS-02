#pragma once

#include "pros/motors.hpp"

extern pros::Motor LadyBrownMech;
extern void liftControl();

void spin_lady_brown_driver(int ladyBrownUpButtonValue, int ladyBrownDownButtonValue);