#pragma once
#include "intake.hpp"
#include "main.h"
#include "pros/adi.hpp"

extern pros::adi::Pneumatics BackMogoActuator;
extern pros::adi::Pneumatics IntakeActuator;


extern bool back_mogo_actuated_value;
extern bool intake_actuated_value;

void actuate_back_mogo(bool out_value);
void actuate_intake(bool out_value);