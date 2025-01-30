#pragma once
#include "pros/colors.h"
#include "pros/motors.hpp"

extern pros::Motor Intake;
extern pros::Motor RingLift;

void spin_intake_auto(bool intaking, int velocity, bool just_first_stage = false);
void stop_intake_auto();
void spin_intake_driver(int intakeButtonValue, int outtakeButtonValue);
void color_sort();