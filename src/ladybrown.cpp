#include "ladybrown.hpp"
#include <cstdlib>
#include "intake.hpp"
#include "main.h"
#include "pros/rtos.hpp"

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

bool toggleThrottleTargetSpeed = false;

const int numStates = 8;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
// int states[numStates] = {14250, 16100, 23000, 27750};
int states[numStates] = {0, 37800, 53500, 59000, 38300, 51800, 50000, 40000};
int currState = 0;
int ladyBrownTarget = states[0];

double kD = 0.001; // 0.02; 


void score2macro() {
  IntakeInputState = 6;
          pros::delay(100);
          nextState(6);
          IntakeInputState = 0;

          pros::delay(50);

          IntakeInputState = 4;

          pros::delay(500);

          IntakeInputState = 0;

          // pros::delay(150);
          nextState(1);

          // pros::delay(100);
          toggleThrottleTargetSpeed = true;
          kD = 0.0035;

          pros::delay(550);

          toggleThrottleTargetSpeed = false;

          kD = 0.001;


          IntakeInputState = 4;
          pros::delay(250);

          // IntakeInputState = 0;
          // pros::delay(30);
          // IntakeInputState = 4;
          // pros::delay(30);

          // IntakeInputState = 6;
          // pros::delay(100);
          // pros::delay(200);

          nextState(6);
          

          IntakeInputState = 0;

          pros::delay(500);

          // pros::delay(1000);
}

void nextState(int theNextState) {
  if (theNextState == -1) {
    currState += 1;
    if (currState >= (numStates - 5)) {
        currState = 0;
    }
    ladyBrownTarget = states[currState];
  } else if (theNextState == 10) {
    score2macro();
  } else {
    ladyBrownTarget = states[theNextState];
    currState=theNextState;
  }
}


double last_error = 0;

double correct360Error(double desiredValue, double sensorValue, int currentStateTarget) {
    // """
    // Corrects the error in the Gyro sensor to make sure
    // that the turns using the gyro sensor are always in
    // the right direction.
    // """
    double tError = desiredValue - sensorValue;
    double newtError = 0;

    // if (desiredValue == 0) {
    //     return tError;
    // } //|| (desiredValue == states[0] 

    if ((tError > 18000 ))
        newtError = tError - 36000;
    else if (tError < -18000)
        newtError =  tError + 36000;
    else
        newtError = tError;
    
    if (currentStateTarget == 0) 
      return -(std::abs(newtError));    
    else
        return newtError;
}

double throttleTargetSpeed(double inputVelocity, double throttlePercentage) {
  return (throttlePercentage * inputVelocity);
}

double velocity = 0;



void liftControl() {
    double kP = 0.02;  // testing value = 0.007// 0.02; // 0.024;
    kD =kD;
    double error =  correct360Error(ladyBrownTarget,  LadyBrownRotationSensor.get_position(), currState);//target-LadyBrownRotationSensor.get_position();
    
    double derivative = (error-last_error);
    if (toggleThrottleTargetSpeed) {
      velocity = throttleTargetSpeed((kP * error + kD * derivative), 0.4);
    } else {
      velocity = (kP * error + kD * derivative);
    }
    last_error = error;
    // if (LadyBrownRotationSensor.get_position() > states[numStates] + 50){

    // }

    string screen_text = std::to_string(LadyBrownRotationSensor.get_position());
    // string screen_text = std::to_string(currState);
    ez::screen_print(screen_text, 6);

    screen_text = std::to_string(error);
    ez::screen_print("error: "+ screen_text, 7);

    
    LadyBrownMech.move(velocity);
}

