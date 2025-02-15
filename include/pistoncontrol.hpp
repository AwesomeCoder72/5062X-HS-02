#pragma once
#include "intake.hpp"
#include "main.h"
#include "pros/adi.hpp"

extern pros::adi::Pneumatics BackMogoActuator;
extern pros::adi::Pneumatics IntakeActuator;
extern pros::adi::Pneumatics LeftDoinkerActuator;
extern pros::adi::Pneumatics RightDoinkerActuator;

extern bool back_mogo_actuated_value;
extern bool intake_actuated_value;

extern bool left_doinker_actuated_value;
extern bool right_doinker_actuated_value;

void actuate_back_mogo(bool out_value);
void actuate_intake(bool up_value);
void actuate_left_doinker(bool out_value);
void actuate_right_doinker(bool out_value);