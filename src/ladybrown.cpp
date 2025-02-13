#include "ladybrown.hpp"
#include "main.h"

void spin_lady_brown_driver(int ladyBrownUpButtonValue, int ladyBrownDownButtonValue) {

  if (((ladyBrownUpButtonValue == 1) && (ladyBrownDownButtonValue == 1)) || 
      ((ladyBrownUpButtonValue == 0) && (ladyBrownDownButtonValue == 0)))  {
        LadyBrownMech.move_voltage(0);
      } else if (ladyBrownUpButtonValue == 1) {
        LadyBrownMech.move_voltage(12000);
      } else if (ladyBrownDownButtonValue == 1) {
        LadyBrownMech.move_voltage(-12000);
      }
}

const int numStates = 4;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {14250, 16100, 23000, 27750};
int currState = 0;
int target = states[0];

void nextState() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
    }
    target = states[currState];
}

double last_error = 0;

void liftControl() {
    double kP = 0.026;
    double kD = 0.0; 
    double error = target - LadyBrownRotationSensor.get_position();
    double derivative = (error-last_error);
    double velocity = kP * error + kD * derivative;
    last_error = error;
    // if (LadyBrownRotationSensor.get_position() > states[numStates] + 50){

    // }
    LadyBrownMech.move(velocity);
}

