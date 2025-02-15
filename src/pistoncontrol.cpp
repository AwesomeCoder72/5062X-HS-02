#include  "pistoncontrol.hpp"
#include "main.h"
#include "pros/adi.hpp"

bool back_mogo_actuated_value = false;

void actuate_back_mogo(bool out_value) {
  BackMogoActuator.set_value(out_value);
  back_mogo_actuated_value = out_value;
}

bool intake_actuated_value = false;

void actuate_intake(bool down_value) {
  IntakeActuator.set_value(down_value);
  intake_actuated_value = down_value;
}

bool actuate_piston(bool out_value, pros::adi::Pneumatics piston) {
  piston.set_value(out_value);
  return out_value;
}

bool left_doinker_actuated_value = false;
bool right_doinker_actuated_value = false;

void actuate_left_doinker(bool out_value) {
  LeftDoinkerActuator.set_value(out_value);
  left_doinker_actuated_value = out_value;
}

void actuate_right_doinker(bool out_value) {
  RightDoinkerActuator.set_value(out_value);
  right_doinker_actuated_value = out_value;
}
