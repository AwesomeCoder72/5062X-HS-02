#include  "pistoncontrol.hpp"
#include "main.h"
#include "pros/adi.hpp"

bool back_mogo_actuated_value = false;

void actuate_back_mogo(bool out_value) {
  BackMogoActuator.set_value(out_value);
  back_mogo_actuated_value = out_value;
}

bool intake_actuated_value = false;

void actuate_intake(bool out_value) {
  IntakeActuator.set_value(out_value);
  intake_actuated_value = out_value;
}

bool actuate_piston(bool out_value, pros::adi::Pneumatics piston) {
  piston.set_value(out_value);
  return out_value;
}

