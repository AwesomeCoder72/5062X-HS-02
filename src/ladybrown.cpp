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
// int states[numStates] = {14250, 16100, 23000, 27750};
int states[numStates] = {21700, 24400, 25700, 0};
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

double correct360Error(double desiredValue, double sensorValue) {
    // """
    // Corrects the error in the Gyro sensor to make sure
    // that the turns using the gyro sensor are always in
    // the right direction.
    // """
    double tError = desiredValue - sensorValue;
    if (tError > 18000 )
        return tError - 36000;
    else if (tError < -18000)
        return tError + 36000;
    else
        return tError;
}

void liftControl() {
    double kP = 0.02; // 0.024;
    double kD = 0.0; // 0.02; 
    double error =  correct360Error(target,  LadyBrownRotationSensor.get_position());//target-LadyBrownRotationSensor.get_position();
    double derivative = (error-last_error);
    double velocity = kP * error + kD * derivative;
    last_error = error;
    // if (LadyBrownRotationSensor.get_position() > states[numStates] + 50){

    // }

    string screen_text = std::to_string(LadyBrownRotationSensor.get_position());
    ez::screen_print(screen_text, 6);

    screen_text = std::to_string(error);
    ez::screen_print(screen_text, 7);

    
    LadyBrownMech.move(velocity);
}

