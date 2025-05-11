#include "autons.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "intake.hpp"
#include "ladybrown.hpp"
#include "main.h"
#include "intake.hpp"
#include "pistoncontrol.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"
#include "drivecontrol.hpp"

// #include 
extern Drive chassis;

// drive_left.


// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {

  // chassis.pid_drive_constants_set(19.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  // chassis.pid_heading_constants_set(2.0, 0.0, 15.0);        // Holds the robot straight while going forward without odom

  chassis.pid_drive_constants_set(20.0, 0.0, 105.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(5.0, 0.0, 10.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5); 

  // P, I, D, and Start I
  // chassis.pid_drive_constants_set(10.7, 0.0, 15.7)
  // ;         // Fwd/rev constants, used for odom and non odom motions
  // chassis.pid_heading_constants_set(12.0, 0.0, 17.5);        // Holds the robot straight while going forward without odom
  // chassis.pid_turn_constants_set(3.0, 0.05, 25.0, 15.0);     // Turn in place constants
  // chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  // chafssis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  // chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(70_ms, 1.5_in, 200_ms, 3.5_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constantsZtg6
  /*:>////;lllllll;llllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll=]yt-6=[rtg5[-=p.;,/
  87yu87]];*/ // This is a random string to make sure the code is still working // This is from the cat, that explanation was from copilotop0-
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.65);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

// double correctGyroError(double desiredValue, double heading) {
//     // """
//     // Corrects the error in the Gyro sensor to make sure
//     // that the turns using the gyro sensor are always in
//     // the right direction.
//     // """
//     double tError = desiredValue - heading;
//     if (tError > 180) return tError - 360;
//     else if (tError < -180) return tError + 360;
//     else return tError;
// }

// void driveStraightPID(double desiredDistance, double desiredHeading, double maxVelocity) {    
//     // Drives straight using a PID loop for distance traveled,
//     // and a PID loop for gyro error correction.
//     // """
//     double maxError = 2;
//     double maxDerivative = 0.5;
//     double dP = 0.7;
//     double dI = 0.0;
//     double dD = 0.0;
    
//     double tP = 0.7;
//     double tI = 0.0;
//     double tD = 0.3;
    
//     double tError = desiredHeading;
//     double tPrevError = tError;
//     double tDerivative = desiredHeading;
//     double dError = desiredDistance;
//     double dPrevError = dError;
//     double dDerivative = desiredDistance;
    
//     double heading = chassis.odom_theta_get();
//     // # tError = desiredHeading
//     // # dError = desiredDistance
    
//     bool finished = false;
    
//     while (!finished) {
//         if ((std::abs(tError)) < (maxError) && abs(tDerivative) < maxDerivative) {
//             if ((desiredDistance) - drive_rt 3) {
//                 finished = true;
//             }
        
//         heading = chassis.odom_theta_get(); // Inertial.heading()
//         // print round(driveLeft.rotation(), 2), round(driveRight.rotation(), 2), round(heading, 2), round(dError, 2), round(tError, 2)
//         tError = correctGyroError(desiredHeading, heading);
//         dError = desiredDistance - (driveLeft.rotation() + driveRight.rotation()) / 2;
        
//         dDerivative = (dError - dPrevError);
//         tDerivative = (tError - tPrevError);
        
//         double dOutput = (dError * dP + dDerivative * dD);
//         double tOutput = (tError * tP + tDerivative * tD);
        
//         chassis.drive_set(dOutput - tOutput, dOutput + tOutput);

//         // driveLeft.spin(vex.DirectionType.FWD, throttleValues(dOutput - tOutput, maxVelocity))
//         // driveRight.spin(vex.DirectionType.FWD, throttleValues(dOutput + tOutput, maxVelocity))
        
//         dPrevError = dError;
//         tPrevError = tError;

//         pros::delay(ez::util::DELAY_TIME);
//                 }
//     }
        
//     chassis.drive_set(0, 0);

// }
///
// Drive Example
///

void blue_left_center_ring_grab_awp() {
  chassis.odom_theta_set(60);

  

  // first ring stack

  chassis.pid_drive_set(15_in, 45, true);
  // chassis.pid_wait_until(5);


  Intake.move_relative(100000, 600);

  // RingLift.move_relative(100000, 300);

  // pros::delay(815);

  // pros::delay(700);


  chassis.pid_wait_until(13);

  actuate_intake(true);

  // Intake.move_relative(100000, 600);


  chassis.pid_wait();

  stop_intake_auto();

  chassis.pid_turn_set(0, 55);
  
  chassis.pid_wait();

  pros::delay(250); // temporary

  // score first ring on alliance stake
  chassis.pid_drive_set(-9.5, 75, true);

  chassis.pid_wait_until(-6);

  actuate_intake(true);
  // actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();

  pros::delay(260);

  chassis.pid_drive_set(13.5, 80, true);
  chassis.pid_wait_until(8);
  stop_intake_auto();
  chassis.pid_wait();

  chassis.pid_turn_set(135, TURN_SPEED);
  // pros::delay(100);
  
  chassis.pid_wait();

  chassis.pid_drive_set(-34, 90, true);
  chassis.pid_wait_until(-30);
  actuate_back_mogo(true);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  chassis.pid_turn_set(56, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(17, 90, true);
  chassis.pid_wait_until(6);
  stop_intake_auto();
  actuate_intake(false);
  chassis.pid_wait();

  actuate_left_doinker(true);
  actuate_left_doinker(false);


  pros::delay(250);
  
  chassis.pid_swing_set(ez::RIGHT_SWING, 26_deg, 50);
  chassis.pid_wait();

  chassis.pid_drive_set(1.5, 60, true);
  chassis.pid_wait();

  actuate_right_doinker(true);
  actuate_right_doinker(true);


  pros::delay(250);

  chassis.pid_swing_set(ez::RIGHT_SWING, 40_deg, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(-38, DRIVE_SPEED, true);

  
  chassis.pid_wait_until(-26);
  actuate_left_doinker(true);

  chassis.pid_wait_until(-30);
  actuate_right_doinker(false);

  chassis.pid_wait();

  chassis.pid_turn_set(80, TURN_SPEED);
  actuate_intake(true);
  chassis.pid_wait();

  chassis.pid_drive_set(8, DRIVE_SPEED, false);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -90, 60);
  spin_intake_auto(true, 600);

  chassis.pid_wait();

  chassis.pid_drive_set(22, 100, true);
  chassis.pid_wait();
  
  pros::delay(2000);

  actuate_back_mogo(false);

  chassis.pid_drive_set(4, 100, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 90);
  chassis.pid_wait();
}

void red_right_center_ring_grab_awp() {
  chassis.odom_theta_set(-60);

  // first ring stack

  chassis.pid_drive_set(15_in, 45, true);
  // chassis.pid_wait_until(5);


  Intake.move_relative(100000, 600);

  // RingLift.move_relative(100000, 300);

  // pros::delay(815);

  // pros::delay(700);


  chassis.pid_wait_until(13);

  actuate_intake(true);

  // Intake.move_relative(100000, 600);


  chassis.pid_wait();

  stop_intake_auto();

  chassis.pid_turn_set(0, 55);
  
  chassis.pid_wait();

  pros::delay(250); // temporary

  // score first ring on alliance stake
  chassis.pid_drive_set(-9.5, 85, true);

  chassis.pid_wait_until(-6);

  actuate_intake(true);
  // actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();

  pros::delay(260);

  chassis.pid_drive_set(13.5, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-135, TURN_SPEED);
  pros::delay(200);
  stop_intake_auto();
  chassis.pid_wait();

  chassis.pid_drive_set(-34, 90, true);
  chassis.pid_wait_until(-30);
  actuate_back_mogo(true);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  chassis.pid_turn_set(-56, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(17, 90, true);
  chassis.pid_wait_until(6);
  stop_intake_auto();
  actuate_intake(false);
  chassis.pid_wait();

  actuate_right_doinker(true);
  actuate_right_doinker(true);


  pros::delay(250);
  
  chassis.pid_swing_set(ez::LEFT_SWING, -26_deg, 50);
  chassis.pid_wait();

  chassis.pid_drive_set(1.5, 60, true);
  chassis.pid_wait();

  actuate_left_doinker(true);
  actuate_left_doinker(false);

  pros::delay(150);

  chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(-38, DRIVE_SPEED, true);

  
  chassis.pid_wait_until(-26);
  actuate_right_doinker(false);

  chassis.pid_wait_until(-30);
  actuate_left_doinker(true);

  chassis.pid_wait();

  chassis.pid_turn_set(-80, TURN_SPEED);
  actuate_intake(true);
  chassis.pid_wait();

  chassis.pid_drive_set(4, DRIVE_SPEED, false);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 90, 60);
  spin_intake_auto(true, 600);

  chassis.pid_wait();

  chassis.pid_drive_set(14, 100, true);
  chassis.pid_wait();

  pros::delay(2000);

  actuate_back_mogo(false);

  chassis.pid_drive_set(4, 100, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 90);
  chassis.pid_wait();
}

void state_skillz() {
  chassis.odom_xyt_set(8.1_in, -59.5_in, 237.25_deg);
  
  ColorSorterToggle = 2;

  chassis.slew_odom_reenable(true);

  // score alliance stake

  actuate_intake(true);

 nextState(2);
 pros::delay(900);
 nextState(0);

  chassis.pid_odom_set({{{20_in, -52_in}, rev, 55},
                                      {{24_in, -48_in}, rev, 40}});
  chassis.pid_wait_until({20_in, -52_in});

  actuate_back_mogo(true);

  chassis.pid_wait();

  pros::delay(300);

  IntakeInputState = 1;

  chassis.pid_odom_set(
   { {{24_in, -28_in}, fwd, 80},
                {{60_in, -3_in}, fwd, 80},
                    {{48_in, 24_in}, fwd, 65}}
              );
  chassis.pid_wait();

  chassis.pid_odom_set(
    
                 {{48_in, 2_in}, fwd, 80}
                //  {{60_in, 0_in, 90_deg}, fwd, 50}
                
               );

  pros::delay(400);  
  IntakeInputState = 5;


  // chassis.pid_wait_until(4_in);

  nextState(1);
  

  
  chassis.pid_wait();

  // chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, 60);
  // chassis.pid_wait();

  // chassis.pid_turn_set(90, 45);
  // chassis.pid_wait();
  

  // chassis.pid_odom_set({{50_in, 0_in}, fwd, 80});

  // chassis.pid_odom_set({{59.5_in, 0_in}, fwd, 50});

  chassis.pid_turn_set(90, 70);
  chassis.pid_wait();

  chassis.pid_drive_set(9.3, 55, true, false);
  IntakeInputState = 1;
  chassis.pid_wait();

  chassis.pid_turn_set(90, 40);
  chassis.pid_wait();

  IntakeInputState = 0;
  pros::delay(30);
  IntakeInputState = 1;
  pros::delay(30);
  IntakeInputState = 0;
  pros::delay(30);
  IntakeInputState = 1;
  pros::delay(30);

  chassis.pid_wait();

  intakeThrottle = 0.5;
  IntakeInputState = 2;

  nextState(2);

  pros::delay(900);

  chassis.pid_odom_set(-12_in, 70);
  intakeThrottle = defaultIntakeThrottle;
  IntakeInputState = 1;
  nextState(0);

  chassis.pid_wait();

  chassis.pid_odom_set({
      {{48_in, -24_in}, fwd, 60}});

  chassis.pid_wait();
  pros::delay(600);

  chassis.pid_odom_set({{45_in, -59_in, 180_deg}, fwd, 50});

  chassis.pid_wait();

  chassis.pid_odom_set({{36_in, -48_in}, rev, 70});
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{59_in, -48_in}, fwd, 70},
  {{48_in, -48_in}, rev, 70}}
  );
  chassis.pid_wait();

  // chassis.pid_turn_set(-45, 70);
  // chassis.pid_wait();

  pros::delay(1000);


  chassis.pid_odom_set({{57_in, -58_in}, rev, 60});
  IntakeInputState = 2;
  chassis.pid_wait();


  actuate_back_mogo(false);

  pros::delay(50);

  chassis.pid_odom_set({{{57_in, 16_in}, fwd, 80}, 
    {{61_in, 42_in, 10_deg}, fwd, 55}});
  IntakeInputState = 0;

  chassis.pid_wait_until(70);
  IntakeInputState = 5;
  chassis.pid_wait();

  chassis.pid_odom_set({
    {{44_in, 36_in}, rev, 80},
    {{14_in, 36_in}, rev, 80}
  });
  chassis.pid_wait();

  // chassis.pid_turn_set(180, 70);
  // chassis.pid_wait();

  chassis.pid_odom_set({{{22_in, 56_in}, rev,80},
  {{50_in, 62_in}, rev, 80}
  });
  chassis.pid_wait();

  chassis.pid_odom_set({{24_in, 50_in}, fwd, 65});
  chassis.pid_wait();

  chassis.pid_odom_set({{{3_in, 50_in}, rev, 60},
    {{0_in, 50_in}, rev, 45}});

  nextState(1);

  chassis.pid_wait_until({3_in, 48_in});
  actuate_back_mogo(true);
  chassis.pid_wait();

  pros::delay(200);

  IntakeInputState = 1;


  // chassis.pid_turn_set(0, 60);
  // chassis.pid_wait();

  chassis.pid_drive_set(5.5, 55, true, false);
  chassis.pid_wait();


  chassis.pid_turn_set(0, 70);
  chassis.pid_wait();

  chassis.pid_drive_set(9.5, 55, true, false);

  // chassis.pid_odom_set({{{4_in, 48_in}, fwd, 80}, {{4_in, 60_in}, fwd, 80}});
  
  nextState(1);
  pros::delay(250);

  IntakeInputState = 0;
  pros::delay(30);
  IntakeInputState = 1;
  pros::delay(50);
  IntakeInputState = 0;
  pros::delay(30);
  IntakeInputState = 1;
  pros::delay(50);
  
  chassis.pid_wait();

  // pros::delay(300);

  // chassis.pid_odom_set(2_in, 60);
  // chassis.pid_wait();

  // chassis.pid_drive_set(-6, 55, true, false);

  

  toggleThrottleTargetSpeed = true;
  // chassis.pid_wait();


  intakeThrottle = 0.5;

  IntakeInputState = 0;


  nextState(4);
  

  pros::delay(1700);

  nextState(2);

  pros::delay(250);


  toggleThrottleTargetSpeed = false;

  chassis.pid_odom_set(-3_in, 60);
  chassis.pid_wait_until(-1);

  IntakeInputState = 0;

  chassis.pid_wait();

  nextState(0);


  intakeThrottle = defaultIntakeThrottle;

  
  IntakeInputState = 1;

  // chassis.pid_odom_set({{36_in, 36_in}, fwd, 70});
  // chassis.pid_wait();

  // IntakeInputState = 1;

  chassis.pid_odom_set({{24_in, 24_in}, fwd, 65});
  chassis.pid_wait();

  pros::delay(1150);

  IntakeInputState = 3;

  chassis.pid_odom_set({{{4_in, 4_in}, fwd, 80}, 
    {{-2_in, -2_in}, fwd, 80}});
    
    chassis.pid_wait_until({4, 4});
    IntakeInputState = 1;
    chassis.pid_wait();


  pros::delay(1200);

  IntakeInputState = 3;


  chassis.pid_odom_set({{-24_in, -20_in}, fwd, 80});
  chassis.pid_wait();

    IntakeInputState = 1;
    
  chassis.pid_odom_set({{{-48_in, -20_in}, fwd, 80},
    // {{-56_in, -47_in}, fwd, 70}
    });
  chassis.pid_wait();

  pros::delay(750);

  // chassis.pid_odom_set(-12_in, 70);
  // chassis.pid_wait();

  // chassis.pid_odom_set({{-48_in, -59_in}, fwd, 50});
  // chassis.pid_wait();

  chassis.pid_odom_set({{-48_in, -54_in}, fwd, 80});
  chassis.pid_wait();

  chassis.pid_odom_set({{-58_in, -58_in}, rev, 80});
  pros::delay(350);
  IntakeInputState = 5;
  chassis.pid_wait();

  IntakeInputState = 2;
  

  actuate_back_mogo(false);

  chassis.pid_odom_set({{-48_in, -48_in}, fwd, 60});
  pros::delay(100);
  IntakeInputState = 5;
  chassis.pid_wait();

  chassis.pid_odom_set({{{-26_in, -49_in}, rev, 70},
    {{-20_in, -50_in}, rev, 70}});
    
  chassis.pid_wait_until({-26_in, -49_in});
  actuate_back_mogo(true);
  chassis.pid_wait();

  pros::delay(100);


  chassis.pid_odom_set({{-56_in, 0_in}, fwd, 70});
  chassis.pid_wait();

  IntakeInputState = 5;

  
  // chassis.pid_odom_set({{-60_in, 0_in, -90_deg}, fwd, 60});
  // chassis.pid_wait();

  chassis.pid_turn_set(-90, 60);
  nextState(1);
  // pros::delay(450);
  IntakeInputState = 1;
  chassis.pid_wait();

  chassis.pid_drive_set(10, 55, true, false);

  // chassis.pid_odom_set({{-62_in, 0_in}, fwd, 70});
  
  chassis.pid_wait();

  pros::delay(500);

  IntakeInputState = 0;
  pros::delay(30);
  IntakeInputState = 1;
  pros::delay(30);
  IntakeInputState = 0;
  pros::delay(30);
  IntakeInputState = 1;
  pros::delay(30);

  toggleThrottleTargetSpeed = true;
  chassis.pid_wait();

  intakeThrottle = 0.5;
  IntakeInputState = 2;


  nextState(2);
  

  pros::delay(600);

  toggleThrottleTargetSpeed = false;

  chassis.pid_odom_set(-3_in, 60);
  chassis.pid_wait_until(-1);

  nextState(0);

  intakeThrottle = defaultIntakeThrottle;
  IntakeInputState = 0;
  chassis.pid_wait();
  
  IntakeInputState = 1;

  chassis.pid_odom_set({{{-54_in, 0_in}, rev, 60},
    {{-48_in, 24_in, }, fwd, 70},
    // {{-62_in, 40_in, }, fwd, 60},
    // {{-48_in, 24_in}, rev, 60},
    // {{-30_in, 22_in}, fwd, 60},
  });
  chassis.pid_wait();

  chassis.pid_odom_set({{-57_in, 26_in}, rev, 70});
  chassis.pid_wait();
    
  chassis.pid_odom_set({{{-65_in, 40_in}, rev, 80}, 
    {{-66_in, 48_in}, rev, 70}, 
    {{-68_in, 56_in}, rev, 60}});
  chassis.pid_wait_until({-65_in, 40_in});
  actuate_back_mogo(false);
  
  
    chassis.pid_wait();


  IntakeInputState = 2;

  chassis.pid_wait();
  // actuate_back_mogo(false);

  chassis.pid_odom_set(12_in, 60);
  chassis.pid_wait();
  
}

void state_full_awp_base() {
  chassis.odom_xyt_set(-11.8_in, -55_in, -237.25_deg);

  chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
  
  ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue

  chassis.slew_odom_reenable(true);

  chassis.pid_odom_set(5.55_in, 80);
  chassis.pid_wait();

  actuate_intake(true);

  nextState(2);
  pros::delay(650);
  nextState(0);

  chassis.pid_odom_set(-6_in, 85);
  chassis.pid_wait();

  chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants


  chassis.pid_turn_set((158), 80, true);
  chassis.pid_wait();

  chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants


  chassis.pid_odom_set(-20_in, 80, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-15_in, 50);

  chassis.pid_wait_until(-11_in);
  
  actuate_back_mogo(true);

  chassis.pid_wait();

  chassis.pid_turn_set(-46, 90);
  chassis.pid_wait();

  IntakeInputState = 1;

  chassis.pid_drive_set(20.5, 90, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-23.5, 90, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-90, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(23, 90, true);
  chassis.pid_wait();

  IntakeInputState = 0;

  chassis.pid_turn_set((118), 80);
  chassis.pid_wait_until(180);
  IntakeInputState = 1;
  chassis.pid_wait();

  chassis.pid_drive_set(30, 95, true);
  chassis.pid_wait_quick_chain();
  actuate_back_mogo(false);
  chassis.pid_drive_set(35, 45, true);
  chassis.pid_wait_until(30);
  IntakeInputState = 5;
  chassis.pid_wait();

  chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants

  chassis.pid_turn_set((197), 80);

  chassis.pid_wait();

  chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants

  chassis.pid_drive_set(-22, 80, true);

  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-15_in, 50);

  chassis.pid_wait_until(-11_in);
  actuate_back_mogo(true);
  chassis.pid_wait();

  

  chassis.pid_turn_set(90, 80);
  IntakeInputState = 1;
  chassis.pid_wait();

  chassis.pid_drive_set(24, 90, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-80, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(26, 90, true);
  chassis.pid_wait_until(5);
  nextState(5);
  chassis.pid_wait();

  chassis.drive_set(0, 0);


  









}

void state_full_awp_red() {
  state_full_awp_base();
}



// void state_full_awp_blue() {
//   chassis.odom_y_flip();
//   chassis.odom_x_flip();
//   chassis.odom_theta_flip();

//   state_full_awp_base();
// }

void state_full_awp_blue() {
  chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);

  chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
  
  ColorSorterToggle = 1; // 1 = throw out red, 2 = throw out blue

  chassis.slew_odom_reenable(true);

  chassis.pid_odom_set(5.55_in, 80);
  chassis.pid_wait();

  actuate_intake(true);

  nextState(2);
  pros::delay(650);
  nextState(0);

  chassis.pid_odom_set(-6_in, 85);
  chassis.pid_wait();

  // chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants


  chassis.pid_turn_set(-158, 80, true);
  chassis.pid_wait();

  // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants


  chassis.pid_odom_set(-20_in, 80, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-15_in, 50);

  chassis.pid_wait_until(-11_in);
  
  actuate_back_mogo(true);

  chassis.pid_wait();

  chassis.pid_turn_set(46, 90);
  chassis.pid_wait();

  IntakeInputState = 1;

  chassis.pid_drive_set(20.5, 90, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-23.5, 90, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(23, 90, true);
  chassis.pid_wait();

  IntakeInputState = 0;

  chassis.pid_turn_set((-118), 80);
  chassis.pid_wait_until(180);
  IntakeInputState = 1;
  chassis.pid_wait();

  chassis.pid_drive_set(30, 95, true);
  chassis.pid_wait_quick_chain();
  actuate_back_mogo(false);
  chassis.pid_drive_set(35, 45, true);
  chassis.pid_wait_until(30);
  IntakeInputState = 5;
  chassis.pid_wait();

  chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants

  chassis.pid_turn_set((-197), 80);

  chassis.pid_wait();

  chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants

  chassis.pid_drive_set(-22, 80, true);

  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-15_in, 50);

  chassis.pid_wait_until(-11_in);
  actuate_back_mogo(true);
  chassis.pid_wait();

  

  chassis.pid_turn_set(-90, 80);
  IntakeInputState = 1;
  chassis.pid_wait();

  chassis.pid_drive_set(24, 90, true);
  chassis.pid_wait();

  chassis.pid_turn_set(80, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(26, 90, true);
  chassis.pid_wait_until(5);
  nextState(5);
  chassis.pid_wait();


  chassis.drive_set(0, 0);
}


void state_elim_goal_red() { 
  
chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);

chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);

ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue

chassis.slew_odom_reenable(true);

chassis.pid_odom_set(5.55_in, 80);
chassis.pid_wait();


nextState(2);
pros::delay(650);
nextState(0);

chassis.pid_odom_set(-6_in, 85);
chassis.pid_wait();

chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants


chassis.pid_turn_set(-158, 80, true);
chassis.pid_wait();

chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants


chassis.pid_odom_set(-15_in, 70, true);
chassis.pid_wait_quick_chain();
chassis.pid_odom_set(-20_in, 45);

chassis.pid_wait_until(-14_in);

actuate_back_mogo(true);

chassis.pid_wait();




chassis.pid_turn_set(-53, TURN_SPEED);
chassis.pid_wait();

chassis.pid_drive_set(15, 90, true);
chassis.pid_wait_until(6);

// chassis.pid_wait_until(3);


chassis.pid_wait();
actuate_right_doinker(true);

pros::delay(250);

// chassis.pid_turn_set(-22, TURN_SPEED);
// chassis.pid_wait();

chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 200_ms);


chassis.pid_swing_set(ez::LEFT_SWING, -22_deg, 40);
chassis.pid_wait(); 

chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);


chassis.pid_drive_set(1.5, 60, true);
chassis.pid_wait_until(0.75);
actuate_left_doinker(true);

chassis.pid_wait();

pros::delay(150);



chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
chassis.pid_wait();

chassis.pid_drive_exit_condition_set(90_ms, 4_in, 250_ms, 7_in, 250_ms, 250_ms);

chassis.pid_drive_set(-43, 110, true);
actuate_intake(true);



chassis.pid_wait_until(-28);
actuate_left_doinker(false);

chassis.pid_wait_until(-29);
actuate_right_doinker(false);

chassis.pid_wait();

chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);


chassis.pid_turn_set(-70, TURN_SPEED);
actuate_intake(true);
chassis.pid_wait();

chassis.pid_drive_set(9, 110, false);
IntakeInputState = 1;
chassis.pid_wait();

chassis.pid_swing_set(ez::LEFT_SWING, 75, 75);

actuate_intake(true);

chassis.pid_wait();

chassis.pid_drive_set(22, 85, true);
chassis.pid_wait();

pros::delay(250);


chassis.pid_drive_set(-6, 85, true);
chassis.pid_wait();

pros::delay(1400);

actuate_back_mogo(false);

chassis.pid_drive_set(3, 110, true);
chassis.pid_wait();


// chassis.pid_turn_set(0, 100);
// chassis.pid_wait();



// chassis.pid_drive_set(7, 110, true);
// chassis.pid_wait();

chassis.pid_turn_set(190, 100, true);
chassis.pid_wait();


}

void state_elim_goal_red_with_center_ring() { 
  
  chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);
  
  chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
  
  ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue
  
  chassis.slew_odom_reenable(true);

  pros::delay(1000);
  
  chassis.pid_odom_set(5.55_in, 80);
  chassis.pid_wait();
  
  
  nextState(2);
  pros::delay(500);
  nextState(0);
  
  chassis.pid_odom_set(-9_in, 100);
  chassis.pid_wait();
  
  chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
  
  
  // chassis.pid_turn_set(-90, 80, true);
  // chassis.pid_wait();
  
  chassis.pid_turn_set(-62.5, 90, true);
  chassis.pid_wait();

  chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants

  // chassis.pid_odom_set(4_in, 85);
  // chassis.pid_wait();



  chassis.pid_odom_set(11, 80, true);
  chassis.pid_wait_until(4);
  IntakeInputState = 5;

  chassis.pid_wait_until(9);

  actuate_intake(true);


  chassis.pid_wait();


  pros::delay(125);

  chassis.pid_odom_set(-6, 70, true);
  chassis.pid_wait();



  // chassis.pid_odom_set(2_in, 80);
  // chassis.pid_wait();

  // actuate_right_doinker(true);
  // actuate_intake(true);
  // pros::delay(400);

  // chassis.pid_odom_set(-12, 70, true);
  // chassis.pid_wait();

  // actuate_right_doinker(false);
  // chassis.pid_wait();

  // chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
  
  // chassis.pid_turn_set(-75, 80, true);
  // chassis.pid_wait();
  
  // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants

  // IntakeInputState = 5;

  // chassis.pid_odom_set(10, 70, true);
  // chassis.pid_wait();

  chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
  
  // chassis.pid_turn_set(-150, 80, true);
  // chassis.pid_wait();
  
  chassis.pid_turn_set(-150, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  
  
  chassis.pid_odom_set(-19_in, 70, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-8_in, 45);
  
  chassis.pid_wait_until(-4_in);
  
  actuate_back_mogo(true);
  
  chassis.pid_wait();

  IntakeInputState = 4;
  
  chassis.pid_turn_set(-39, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(18.5, 90, true);
  actuate_intake(false);
  IntakeInputState = 0;
  chassis.pid_wait_until(6);
  
  // chassis.pid_wait_until(3);
  
  
  chassis.pid_wait();
  actuate_right_doinker(true);
  
  pros::delay(250);
  
  // chassis.pid_turn_set(-22, TURN_SPEED);
  // chassis.pid_wait();
  
  chassis.pid_swing_exit_condition_set(50_ms, 4_deg, 125_ms, 8_deg, 250_ms, 200_ms);
  
  
  chassis.pid_swing_set(ez::LEFT_SWING, -13.5_deg, 40);
  chassis.pid_wait(); 
  
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  
  
  chassis.pid_drive_set(2.75, 60, true);
  chassis.pid_wait_until(1.25);
  actuate_left_doinker(true);
  
  chassis.pid_wait();
  
  pros::delay(150);
  
  
  
  chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
  chassis.pid_wait();
  
  chassis.pid_drive_exit_condition_set(90_ms, 4_in, 250_ms, 7_in, 250_ms, 250_ms);
  
  chassis.pid_drive_set(-43, 110, true);
  actuate_intake(true);
  
  
  
  chassis.pid_wait_until(-28);
  actuate_left_doinker(false);
  
  chassis.pid_wait_until(-29);
  actuate_right_doinker(false);
  
  chassis.pid_wait();
  
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  
  
  chassis.pid_turn_set(-60, TURN_SPEED);
  actuate_intake(true);
  chassis.pid_wait();
  
  chassis.pid_odom_set(9, 110, false);
  IntakeInputState = 1;
  chassis.pid_wait();

  chassis.pid_turn_set(15, 80, true);
  chassis.pid_wait();
  
  // chassis.pid_swing_set(ez::LEFT_SWING, 75, 75);
  
  chassis.pid_wait();

  chassis.pid_odom_set(5, 110, false);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 90, 70);
  chassis.pid_wait();

  
  // chassis.pid_drive_set(14, 85, true);
  // chassis.pid_wait();

  // chassis.pid_turn_set(90, 80, true);
  // chassis.pid_wait();

  chassis.pid_odom_set(10, 100, true);
  chassis.pid_wait();
  
  // pros::delay(250);

  chassis.pid_turn_set(0, 100);
  chassis.pid_wait();
  
  
  // chassis.pid_turn_set(0, 100);
  // chassis.pid_wait();
  
  
  
  // chassis.pid_drive_set(7, 110, true);
  // chassis.pid_wait();

  
  
  }

  void state_elim_goal_blue_with_center_ring() { 


  
    chassis.odom_xyt_set(-11.8_in, -55_in, -237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 1; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(5.55_in, 80);
    chassis.pid_wait();
    
    
    nextState(2);
    pros::delay(500);
    nextState(0);
    
    chassis.pid_odom_set(-9_in, 100);
    chassis.pid_wait();
    
    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    
    // chassis.pid_turn_set(-90, 80, true);
    // chassis.pid_wait();
    
    // chassis.pid_turn_set(62.5, 90, true);
    // chassis.pid_wait();
  
    // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  
    // // chassis.pid_odom_set(4_in, 85);
    // // chassis.pid_wait();
  
  
  
    // chassis.pid_odom_set(11, 80, true);
    // chassis.pid_wait_until(4);
    // IntakeInputState = 5;
  
    // chassis.pid_wait_until(9);
  
    // actuate_intake(true);
  
  
    // chassis.pid_wait();
  
  
    // pros::delay(125);
  
    // chassis.pid_odom_set(-6, 70, true);
    // chassis.pid_wait();
  
  
  
    // chassis.pid_odom_set(2_in, 80);
    // chassis.pid_wait();
  
    // actuate_right_doinker(true);
    // actuate_intake(true);
    // pros::delay(400);
  
    // chassis.pid_odom_set(-12, 70, true);
    // chassis.pid_wait();
  
    // actuate_right_doinker(false);
    // chassis.pid_wait();
  
    // chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    // chassis.pid_turn_set(-75, 80, true);
    // chassis.pid_wait();
    
    // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  
    // IntakeInputState = 5;
  
    // chassis.pid_odom_set(10, 70, true);
    // chassis.pid_wait();
  
    // chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    // chassis.pid_turn_set(-150, 80, true);
    // chassis.pid_wait();
    
    // chassis.pid_turn_set(150, 80, true);
    // chassis.pid_wait();
  
    // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
    
    chassis.pid_swing_set(ez::RIGHT_SWING, 163, 70);
    chassis.pid_wait();
    
    chassis.pid_odom_set(-19_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-7_in, 45);
    
    chassis.pid_wait_until(-4_in);
    
    actuate_back_mogo(true);
    
    chassis.pid_wait();
  
    IntakeInputState = 4;
    
    chassis.pid_turn_set(44, TURN_SPEED);
    chassis.pid_wait();
    
    chassis.pid_drive_set(17, 90, true);
    actuate_intake(false);
    IntakeInputState = 0;
    chassis.pid_wait_until(6);
    
    // chassis.pid_wait_until(3);
    
    
    chassis.pid_wait();
    actuate_left_doinker(true);
    
    pros::delay(250);
    
    // chassis.pid_turn_set(-22, TURN_SPEED);
    // chassis.pid_wait();
    
    chassis.pid_swing_exit_condition_set(50_ms, 4_deg, 125_ms, 8_deg, 250_ms, 200_ms);
    
    
    chassis.pid_swing_set(ez::RIGHT_SWING, 17_deg, 40);
    chassis.pid_wait(); 
    
    chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    
    
    chassis.pid_drive_set(2.75, 60, true);
    chassis.pid_wait_until(1.25);
    actuate_right_doinker(true);
    
    chassis.pid_wait();
    
    pros::delay(150);
    
    
    
    chassis.pid_swing_set(ez::RIGHT_SWING, 40_deg, 60);
    chassis.pid_wait();
    
    chassis.pid_drive_exit_condition_set(60_ms, 5_in, 200_ms, 7_in, 250_ms, 250_ms);
    
    chassis.pid_drive_set(-43, 110, true);
    actuate_intake(true);
    
    
    
    chassis.pid_wait_until(-28);
    actuate_right_doinker(false);
    
    chassis.pid_wait_until(-29);
    actuate_left_doinker(false);
    
    chassis.pid_wait();
    
    chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
    
    
    chassis.pid_turn_set(60, TURN_SPEED);
    actuate_intake(true);
    chassis.pid_wait();
    
    chassis.pid_odom_set(9, 110, false);
    IntakeInputState = 1;
    chassis.pid_wait();
  
    chassis.pid_turn_set(-15, 90, true);
    chassis.pid_wait();
    
    // chassis.pid_swing_set(ez::LEFT_SWING, 75, 75);
    
    chassis.pid_wait();
  
    chassis.pid_odom_set(5, 110, false);
    chassis.pid_wait();
  
    chassis.pid_swing_set(ez::RIGHT_SWING, -75, 70);
    chassis.pid_wait();
  
    
    // chassis.pid_drive_set(14, 85, true);
    // chassis.pid_wait();
  
    // chassis.pid_turn_set(90, 80, true);
    // chassis.pid_wait();
  
    chassis.pid_odom_set(10, 100, true);
    chassis.pid_wait();
    
    // pros::delay(250);
  
    chassis.pid_turn_set(0, 100);
    chassis.pid_wait();

    pros::delay(750);

    actuate_back_mogo(false);

    pros::delay(300);

    chassis.pid_turn_set(20, 180);
    chassis.pid_wait();
    
    
    // chassis.pid_turn_set(0, 100);
    // chassis.pid_wait();
    
    
    
    // chassis.pid_drive_set(7, 110, true);
    // chassis.pid_wait();
  
    
    
    }


    void worlds_elim_goal_red_with_center_ring() { 


  
      chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);
      
      chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
      
      ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue
      
      chassis.slew_odom_reenable(true);
    
      // pros::delay(1000);
      
      chassis.pid_odom_set(5.55_in, 80);
      chassis.pid_wait();
      
      
      nextState(2);
      pros::delay(500);
      nextState(0);
      
      chassis.pid_odom_set(-9_in, 100);
      chassis.pid_wait();
      
      chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
      
      
      // chassis.pid_turn_set(-90, 80, true);
      // chassis.pid_wait();
      
      // chassis.pid_turn_set(62.5, 90, true);
      // chassis.pid_wait();
    
      // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
    
      // // chassis.pid_odom_set(4_in, 85);
      // // chassis.pid_wait();
    
    
    
      // chassis.pid_odom_set(11, 80, true);
      // chassis.pid_wait_until(4);
      // IntakeInputState = 5;
    
      // chassis.pid_wait_until(9);
    
      // actuate_intake(true);
    
    
      // chassis.pid_wait();
    
    
      // pros::delay(125);
    
      // chassis.pid_odom_set(-6, 70, true);
      // chassis.pid_wait();
    
    
    
      // chassis.pid_odom_set(2_in, 80);
      // chassis.pid_wait();
    
      // actuate_right_doinker(true);
      // actuate_intake(true);
      // pros::delay(400);
    
      // chassis.pid_odom_set(-12, 70, true);
      // chassis.pid_wait();
    
      // actuate_right_doinker(false);
      // chassis.pid_wait();
    
      // chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
      
      // chassis.pid_turn_set(-75, 80, true);
      // chassis.pid_wait();
      
      // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
    
      // IntakeInputState = 5;
    
      // chassis.pid_odom_set(10, 70, true);
      // chassis.pid_wait();
    
      // chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
      
      // chassis.pid_turn_set(-150, 80, true);
      // chassis.pid_wait();
      
      // chassis.pid_turn_set(150, 80, true);
      // chassis.pid_wait();
    
      // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
      
      chassis.pid_swing_set(ez::LEFT_SWING, -163, 70);
      chassis.pid_wait();
      
      chassis.pid_odom_set(-19_in, 70, true);
      chassis.pid_wait_quick_chain();
      chassis.pid_odom_set(-7_in, 45);
      
      chassis.pid_wait_until(-4_in);
      
      actuate_back_mogo(true);
      
      chassis.pid_wait();
    
      IntakeInputState = 4;
      
      chassis.pid_turn_set(-44, TURN_SPEED);
      chassis.pid_wait();
      
      chassis.pid_drive_set(17, 90, true);
      actuate_intake(false);
      IntakeInputState = 0;
      chassis.pid_wait_until(6);
      
      // chassis.pid_wait_until(3);
      
      
      chassis.pid_wait();
      actuate_right_doinker(true);
      
      pros::delay(250);
      
      // chassis.pid_turn_set(-22, TURN_SPEED);
      // chassis.pid_wait();
      
      chassis.pid_swing_exit_condition_set(50_ms, 4_deg, 125_ms, 8_deg, 250_ms, 200_ms);
      
      
      chassis.pid_swing_set(ez::LEFT_SWING, -17_deg, 40);
      chassis.pid_wait(); 
      
      chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
      
      
      chassis.pid_drive_set(2.75, 60, true);
      chassis.pid_wait_until(1.25);
      actuate_left_doinker(true);
      
      chassis.pid_wait();
      
      pros::delay(150);
      
      
      
      chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
      chassis.pid_wait();
      
      chassis.pid_drive_exit_condition_set(60_ms, 5_in, 200_ms, 7_in, 250_ms, 250_ms);
      
      chassis.pid_drive_set(-43, 110, true);
      actuate_intake(true);
      
      
      
      chassis.pid_wait_until(-28);
      actuate_left_doinker(false);
      
      chassis.pid_wait_until(-29);
      actuate_right_doinker(false);
      
      chassis.pid_wait();
      
      chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
      
      
      chassis.pid_turn_set(-60, TURN_SPEED);
      actuate_intake(true);
      chassis.pid_wait();
      
      chassis.pid_odom_set(9, 110, false);
      IntakeInputState = 1;
      chassis.pid_wait();
    
      chassis.pid_turn_set(15, 90, true);
      chassis.pid_wait();
      
      // chassis.pid_swing_set(ez::LEFT_SWING, 75, 75);
      
      chassis.pid_wait();
    
      chassis.pid_odom_set(5, 110, false);
      chassis.pid_wait();
    
      chassis.pid_swing_set(ez::LEFT_SWING, 75, 70);
      chassis.pid_wait();
    
      
      // chassis.pid_drive_set(14, 85, true);
      // chassis.pid_wait();
    
      // chassis.pid_turn_set(90, 80, true);
      // chassis.pid_wait();
    
      chassis.pid_odom_set(-10, 100, true);
      chassis.pid_wait();
      
      // pros::delay(250);
    
      chassis.pid_turn_set(0, 100);
      chassis.pid_wait();
  
      pros::delay(750);
  
      actuate_back_mogo(false);
  
      pros::delay(300);
  
      chassis.pid_turn_set(-20, 180);
      chassis.pid_wait();
      
      
      // chassis.pid_turn_set(0, 100);
      // chassis.pid_wait();
      
      
      
      // chassis.pid_drive_set(7, 110, true);
      // chassis.pid_wait();
    
      
      
      }

  void blue_ring_six_ring() {
    
    chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 1; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(5.55_in, 80);
    chassis.pid_wait();
    
    
    nextState(2);
    pros::delay(500);
    nextState(0);
    
    chassis.pid_odom_set(-9_in, 100);
    chassis.pid_wait();
    
    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    
    // chassis.pid_turn_set(-90, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(-62.5, 90, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  
    // chassis.pid_odom_set(4_in, 85);
    // chassis.pid_wait();
  
  
  
    chassis.pid_odom_set(11, 80, true);
    chassis.pid_wait_until(4);
    IntakeInputState = 5;
  
    chassis.pid_wait_until(9);
  
    actuate_intake(true);
  
  
    chassis.pid_wait();
  
  
    pros::delay(125);
  
    chassis.pid_odom_set(-6, 70, true);
    chassis.pid_wait();

    chassis.pid_turn_set(-150, 80, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
    
    
    chassis.pid_odom_set(-19_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-6_in, 45);
    
    chassis.pid_wait_until(-3_in);
    
    actuate_back_mogo(true);
    
    chassis.pid_wait();
  
    IntakeInputState = 4;
    
    chassis.pid_turn_set(-35, TURN_SPEED);
    chassis.pid_wait();
    
    chassis.pid_drive_set(20, 90, true);
    actuate_intake(false);
    IntakeInputState = 0;
    chassis.pid_wait_until(6);
    
    // chassis.pid_wait_until(3);
    
    
    chassis.pid_wait();
    actuate_right_doinker(true);
    
    pros::delay(250);

    chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
    chassis.pid_wait();
    
    chassis.pid_drive_exit_condition_set(90_ms, 4_in, 250_ms, 7_in, 250_ms, 250_ms);
    
    chassis.pid_odom_set(-15, 110, true);
    actuate_intake(true);
    // chassis.pid_wait_until(-20);
    
    chassis.pid_wait();



    chassis.pid_turn_set(60, 80, true);


    chassis.pid_wait_until(50);
    actuate_right_doinker(false);
    // actuate_left_doinker(false);
    chassis.pid_wait();

    pros::delay(200);

    chassis.pid_turn_set(95, 80);
    chassis.pid_wait();

    IntakeInputState = 1;

    chassis.pid_odom_set(26, 90);
    chassis.pid_wait();

    chassis.pid_turn_set(120, 80, true);
    chassis.pid_wait();

    chassis.pid_odom_set(-20, 90);
    chassis.pid_wait();

    chassis.pid_turn_set(45, 80, true);
    chassis.pid_wait();

    chassis.pid_odom_set(7, 90);
    chassis.pid_wait();

    chassis.pid_swing_set(ez::LEFT_SWING, 91, 70);
    chassis.pid_wait();

    chassis.pid_odom_set(14, 80);
    chassis.pid_wait();

    chassis.pid_swing_set(ez::LEFT_SWING, 50, 70);
    chassis.pid_wait();

    chassis.pid_odom_set(-54, 110);
    chassis.pid_wait();

    chassis.pid_turn_set(-100, 100, true);
    chassis.pid_wait();

    chassis.pid_odom_set(46, 110);
    actuate_left_doinker(true);
    chassis.pid_wait();


    // chassis.pid_swing_set(ez::LEFT_SWING, 90, 70);
    // chassis.pid_wait();

    // chassis.pid_odom_set(8, 80);
    // chassis.pid_wait();

    // chassis.pid_swing_set(ez::LEFT_SWING, 45, 70);
    // chassis.pid_wait();

    // chassis.pid_turn_set(135, 80);
    // chassis.pid_wait();

    // chassis.pid_odom_set(16, 80);
    // chassis.pid_wait();






  }

  void blue_ring_six_ring_new() {
    
    chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 1; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(5.55_in, 80);
    chassis.pid_wait();
    
    
    nextState(2);
    pros::delay(500);
    nextState(0);
    
    chassis.pid_odom_set(-9_in, 100);
    chassis.pid_wait();
    
    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    
    // chassis.pid_turn_set(-90, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(-155, 90, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  
    // chassis.pid_odom_set(4_in, 85);
    // chassis.pid_wait();
  
  
  
    // chassis.pid_odom_set(11, 80, true);
    // chassis.pid_wait_until(4);
    // IntakeInputState = 5;
  
    // chassis.pid_wait_until(9);
  
    // actuate_intake(true);
  
  
    // chassis.pid_wait();
  
  
    // pros::delay(125);
  
    // chassis.pid_odom_set(-6, 70, true);
    // chassis.pid_wait();

    // chassis.pid_turn_set(-150, 80, true);
    // chassis.pid_wait();
  
    // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
    
    
    chassis.pid_odom_set(-19_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-12_in, 45);
    
    chassis.pid_wait_until(-7_in);
    
    actuate_back_mogo(true);
    
    chassis.pid_wait();
  
    IntakeInputState = 4;
    
    chassis.pid_turn_set(-45, TURN_SPEED);
    chassis.pid_wait();
    
    chassis.pid_drive_set(15.75, 90, true);
    actuate_intake(false);
    IntakeInputState = 0;
    chassis.pid_wait_until(6);
    
    // chassis.pid_wait_until(3);
    
    
    chassis.pid_wait();
    actuate_right_doinker(true);
    
    pros::delay(300);

    // chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
    // chassis.pid_wait();
    
    chassis.pid_drive_exit_condition_set(90_ms, 4_in, 250_ms, 7_in, 250_ms, 250_ms);
    
    chassis.pid_odom_set(-13, 110, true);
    actuate_intake(true);
    // chassis.pid_wait_until(-20);
    
    chassis.pid_wait();



    chassis.pid_turn_set(70, 80, true);


    chassis.pid_wait_until(60);
    actuate_right_doinker(false);
    // actuate_left_doinker(false);
    chassis.pid_wait();

    pros::delay(200);

    chassis.pid_turn_set(95, 100);
    chassis.pid_wait();

    IntakeInputState = 1;

    chassis.pid_odom_set(26, 90);
    chassis.pid_wait();

    chassis.pid_turn_set(120, 80, true);
    chassis.pid_wait();

    chassis.pid_odom_set(-20.5, 90);
    chassis.pid_wait();

    chassis.pid_turn_set(45, 80, true);
    chassis.pid_wait();

    chassis.pid_odom_set(2, 90);
    chassis.pid_wait();

    chassis.pid_swing_set(ez::LEFT_SWING, 90, 65);
    chassis.pid_wait();

    chassis.pid_odom_set(14, 80);
    chassis.pid_wait();

    chassis.pid_swing_set(ez::LEFT_SWING, 50, 70);
    chassis.pid_wait();

    chassis.pid_odom_set(-42, 110);
    chassis.pid_wait();

    chassis.pid_turn_set(-102, 100, true);
    actuate_intake(false);
    chassis.pid_wait();

    chassis.pid_odom_set(16, 80);
    // chassis.pid_wait_until(5);
    // actuate_intake(true);
    chassis.pid_wait_quick_chain();

    // chassis.pid_odom_set(-3, 90);
    // chassis.pid_wait();

    chassis.pid_odom_set(39, 110);
    actuate_left_doinker(true);
    chassis.pid_wait();


    // chassis.pid_swing_set(ez::LEFT_SWING, 90, 70);
    // chassis.pid_wait();

    // chassis.pid_odom_set(8, 80);
    // chassis.pid_wait();

    // chassis.pid_swing_set(ez::LEFT_SWING, 45, 70);
    // chassis.pid_wait();

    // chassis.pid_turn_set(135, 80);
    // chassis.pid_wait();

    // chassis.pid_odom_set(16, 80);
    // chassis.pid_wait();






  }

  void red_ring_six_ring_new() {
    
    chassis.odom_xyt_set(-11.8_in, -55_in, -237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(5.55_in, 80);
    chassis.pid_wait();
    
    
    nextState(2);
    pros::delay(500);
    nextState(0);
    
    chassis.pid_odom_set(-9_in, 100);
    chassis.pid_wait();
    
    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    
    // chassis.pid_turn_set(-90, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(155, 90, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  
    // chassis.pid_odom_set(4_in, 85);
    // chassis.pid_wait();
  
  
  
    // chassis.pid_odom_set(11, 80, true);
    // chassis.pid_wait_until(4);
    // IntakeInputState = 5;
  
    // chassis.pid_wait_until(9);
  
    // actuate_intake(true);
  
  
    // chassis.pid_wait();
  
  
    // pros::delay(125);
  
    // chassis.pid_odom_set(-6, 70, true);
    // chassis.pid_wait();

    // chassis.pid_turn_set(-150, 80, true);
    // chassis.pid_wait();
  
    // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
    
    
    chassis.pid_odom_set(-19_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-12_in, 45);
    
    chassis.pid_wait_until(-7_in);
    
    actuate_back_mogo(true);
    
    chassis.pid_wait();
  
    IntakeInputState = 4;
    
    chassis.pid_turn_set(45, TURN_SPEED);
    chassis.pid_wait();
    
    chassis.pid_drive_set(15.75, 90, true);
    actuate_intake(false);
    IntakeInputState = 0;
    chassis.pid_wait_until(6);
    
    // chassis.pid_wait_until(3);
    
    
    chassis.pid_wait();
    actuate_left_doinker(true);
    
    pros::delay(300);

    // chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
    // chassis.pid_wait();
    
    chassis.pid_drive_exit_condition_set(90_ms, 4_in, 250_ms, 7_in, 250_ms, 250_ms);
    
    chassis.pid_odom_set(-13, 110, true);
    actuate_intake(true);
    // chassis.pid_wait_until(-20);
    
    chassis.pid_wait();



    chassis.pid_turn_set(-70, 80, true);


    chassis.pid_wait_until(60);
    actuate_left_doinker(false);
    // actuate_left_doinker(false);
    chassis.pid_wait();

    pros::delay(200);

    chassis.pid_turn_set(-95, 100);
    chassis.pid_wait();

    IntakeInputState = 1;

    chassis.pid_odom_set(26, 90);
    chassis.pid_wait();

    chassis.pid_turn_set(-120, 80, true);
    chassis.pid_wait();

    chassis.pid_odom_set(-20.5, 90);
    chassis.pid_wait();

    chassis.pid_turn_set(-45, 80, true);
    chassis.pid_wait();

    chassis.pid_odom_set(2, 90);
    chassis.pid_wait();

    chassis.pid_swing_set(ez::RIGHT_SWING, -90, 65);
    chassis.pid_wait();

    chassis.pid_odom_set(14, 80);
    chassis.pid_wait();

    chassis.pid_swing_set(ez::RIGHT_SWING, -50, 70);
    chassis.pid_wait();

    chassis.pid_odom_set(-42, 110);
    chassis.pid_wait();

    chassis.pid_turn_set(102, 100, true);
    actuate_intake(false);
    chassis.pid_wait();

    chassis.pid_odom_set(16, 80);
    // chassis.pid_wait_until(5);
    // actuate_intake(true);
    chassis.pid_wait_quick_chain();

    // chassis.pid_odom_set(-3, 90);
    // chassis.pid_wait();

    chassis.pid_odom_set(39, 110);
    actuate_right_doinker(true);
    chassis.pid_wait();


    // chassis.pid_swing_set(ez::LEFT_SWING, 90, 70);
    // chassis.pid_wait();

    // chassis.pid_odom_set(8, 80);
    // chassis.pid_wait();

    // chassis.pid_swing_set(ez::LEFT_SWING, 45, 70);
    // chassis.pid_wait();

    // chassis.pid_turn_set(135, 80);
    // chassis.pid_wait();

    // chassis.pid_odom_set(16, 80);
    // chassis.pid_wait();






  }

  void red_ring_safe_no_stack() {
    chassis.odom_xyt_set(-11.8_in, -55_in, -237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(-15_in, 85);
    chassis.pid_wait();

    chassis.pid_turn_set(180, 80, true);
    chassis.pid_wait();

    // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants


    chassis.pid_odom_set(-15_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-10_in, 45);

    chassis.pid_wait_until(-7_in);
    
    actuate_back_mogo(true);

    chassis.pid_wait();

    chassis.pid_turn_set(-105, 80);
    chassis.pid_wait();

    actuate_intake(true);

    IntakeInputState = 1;

    chassis.pid_odom_set(26, 75, true);
    chassis.pid_wait();

    chassis.pid_turn_set(65, 80);
    chassis.pid_wait();

    chassis.pid_odom_set(38, 80);
    // chassis.pid_wait_until(12);
    // nextState(3);
    chassis.pid_wait();


  }

  void blue_goal_safe_no_stack() {
    chassis.odom_xyt_set(-11.8_in, -55_in, -237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 1; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(-15_in, 85);
    chassis.pid_wait();

    chassis.pid_turn_set(180, 80, true);
    chassis.pid_wait();

    // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants


    chassis.pid_odom_set(-15_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-10_in, 45);

    chassis.pid_wait_until(-7_in);
    
    actuate_back_mogo(true);

    chassis.pid_wait();

    chassis.pid_turn_set(-105, 80);
    chassis.pid_wait();

    actuate_intake(true);

    IntakeInputState = 1;

    chassis.pid_odom_set(26, 75, true);
    chassis.pid_wait();

    chassis.pid_turn_set(65, 80);
    chassis.pid_wait();

    chassis.pid_odom_set(38, 80);
    // chassis.pid_wait_until(12);
    // nextState(3);
    chassis.pid_wait();


  }

  void red_goal_safe_no_stack() {
    chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(-15_in, 85);
    chassis.pid_wait();

    chassis.pid_turn_set(180, 80, true);
    chassis.pid_wait();

    // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants


    chassis.pid_odom_set(-15_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-10_in, 45);

    chassis.pid_wait_until(-7_in);
    
    actuate_back_mogo(true);

    chassis.pid_wait();

    chassis.pid_turn_set(105, 80);
    chassis.pid_wait();

    actuate_intake(true);

    IntakeInputState = 1;

    chassis.pid_odom_set(26, 75, true);
    chassis.pid_wait();

    chassis.pid_turn_set(-65, 80);
    chassis.pid_wait();

    chassis.pid_odom_set(38, 80);
    // chassis.pid_wait_until(12);
    // nextState(3);
    chassis.pid_wait();


  }

  void blue_ring_safe_no_stack() {
    chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 1; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(-15_in, 85);
    chassis.pid_wait();

    chassis.pid_turn_set(180, 80, true);
    chassis.pid_wait();

    // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants


    chassis.pid_odom_set(-15_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-10_in, 45);

    chassis.pid_wait_until(-7_in);
    
    actuate_back_mogo(true);

    chassis.pid_wait();

    chassis.pid_turn_set(105, 80);
    chassis.pid_wait();

    actuate_intake(true);

    IntakeInputState = 1;

    chassis.pid_odom_set(26, 75, true);
    chassis.pid_wait();

    chassis.pid_turn_set(-65, 80);
    chassis.pid_wait();

    chassis.pid_odom_set(38, 80);
    // chassis.pid_wait_until(12);
    // nextState(3);
    chassis.pid_wait();


  }


  
void blue_goal_safe_alliance_and_stack() {
  chassis.odom_xyt_set(-11.8_in, -55_in, -237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 1; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(5.55_in, 80);
    chassis.pid_wait();
    
    
    nextState(2);
    pros::delay(500);
    nextState(0);
    
    chassis.pid_odom_set(-9_in, 100);
    chassis.pid_wait();
    
    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    
    // chassis.pid_turn_set(-90, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(62.5, 90, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  
    // chassis.pid_odom_set(4_in, 85);
    // chassis.pid_wait();
  
  
  
    chassis.pid_odom_set(11, 80, true);
    chassis.pid_wait_until(4);
    IntakeInputState = 5;
  
    chassis.pid_wait_until(9);
  
    actuate_intake(true);
  
  
    chassis.pid_wait();
  
  
    pros::delay(125);
  
    chassis.pid_odom_set(-6, 70, true);
    chassis.pid_wait();

    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    // chassis.pid_turn_set(-150, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(150, 80, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
    
    
    chassis.pid_odom_set(-19_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-8_in, 45);
    
    chassis.pid_wait_until(-4_in);
    
    actuate_back_mogo(true);
    
    chassis.pid_wait();

    IntakeInputState = 0;

  chassis.pid_turn_set(-85, 80);
  chassis.pid_wait();

  IntakeInputState = 1;

    chassis.pid_odom_set(26, 75, true);
    chassis.pid_wait();

    pros::delay(500);

    chassis.pid_turn_set(70, 80);
    chassis.pid_wait();

    chassis.pid_odom_set(35, 70);
    // chassis.pid_wait_until(12);
    // nextState(3);
    chassis.pid_wait();
}

void red_ring_safe_alliance_and_stack() {
  chassis.odom_xyt_set(-11.8_in, -55_in, -237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(5.55_in, 80);
    chassis.pid_wait();
    
    
    nextState(2);
    pros::delay(500);
    nextState(0);
    
    chassis.pid_odom_set(-9_in, 100);
    chassis.pid_wait();
    
    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    
    // chassis.pid_turn_set(-90, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(62.5, 90, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  
    // chassis.pid_odom_set(4_in, 85);
    // chassis.pid_wait();
  
  
  
    chassis.pid_odom_set(11, 80, true);
    chassis.pid_wait_until(4);
    IntakeInputState = 5;
  
    chassis.pid_wait_until(9);
  
    actuate_intake(true);
  
  
    chassis.pid_wait();
  
  
    pros::delay(125);
  
    chassis.pid_odom_set(-6, 70, true);
    chassis.pid_wait();

    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    // chassis.pid_turn_set(-150, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(150, 80, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
    
    
    chassis.pid_odom_set(-19_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-8_in, 45);
    
    chassis.pid_wait_until(-4_in);
    
    actuate_back_mogo(true);
    
    chassis.pid_wait();

    IntakeInputState = 0;

  chassis.pid_turn_set(-85, 80);
  chassis.pid_wait();

  // IntakeInputState = 1;
  IntakeInputState = 1;

    chassis.pid_odom_set(26, 75, true);
    chassis.pid_wait();

    pros::delay(500);

    chassis.pid_turn_set(70, 80);
    chassis.pid_wait();

    chassis.pid_odom_set(35, 70);
    // chassis.pid_wait_until(12);
    // nextState(3);
    chassis.pid_wait();
}

void blue_ring_safe_alliance_and_stack() {
  chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 1; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(5.55_in, 80);
    chassis.pid_wait();
    
    
    nextState(2);
    pros::delay(500);
    nextState(0);
    
    chassis.pid_odom_set(-9_in, 100);
    chassis.pid_wait();
    
    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    
    // chassis.pid_turn_set(-90, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(-62.5, 90, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  
    // chassis.pid_odom_set(4_in, 85);
    // chassis.pid_wait();
  
  
  
    chassis.pid_odom_set(11, 80, true);
    chassis.pid_wait_until(4);
    IntakeInputState = 5;
  
    chassis.pid_wait_until(9);
  
    actuate_intake(true);
  
  
    chassis.pid_wait();
  
  
    pros::delay(125);
  
    chassis.pid_odom_set(-6, 70, true);
    chassis.pid_wait();

    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    // chassis.pid_turn_set(-150, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(-150, 80, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
    
    
    chassis.pid_odom_set(-19_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-8_in, 45);
    
    chassis.pid_wait_until(-4_in);
    
    actuate_back_mogo(true);
    
    chassis.pid_wait();

    IntakeInputState = 0;

  chassis.pid_turn_set(85, 80);
  chassis.pid_wait();

  IntakeInputState = 1;

    chassis.pid_odom_set(26, 75, true);
    chassis.pid_wait();

    pros::delay(500);

    chassis.pid_turn_set(-70, 80);
    chassis.pid_wait();

    chassis.pid_odom_set(35, 70);
    // chassis.pid_wait_until(12);
    // nextState(3);
    chassis.pid_wait();
}

void red_goal_safe_alliance_and_stack() {
  chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(5.55_in, 80);
    chassis.pid_wait();
    
    
    nextState(2);
    pros::delay(500);
    nextState(0);
    
    chassis.pid_odom_set(-9_in, 100);
    chassis.pid_wait();
    
    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    
    // chassis.pid_turn_set(-90, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(-62.5, 90, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  
    // chassis.pid_odom_set(4_in, 85);
    // chassis.pid_wait();
  
  
  
    chassis.pid_odom_set(11, 80, true);
    chassis.pid_wait_until(4);
    IntakeInputState = 5;
  
    chassis.pid_wait_until(9);
  
    actuate_intake(true);
  
  
    chassis.pid_wait();
  
  
    pros::delay(125);
  
    chassis.pid_odom_set(-6, 70, true);
    chassis.pid_wait();

    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    // chassis.pid_turn_set(-150, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(-150, 80, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
    
    
    chassis.pid_odom_set(-19_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-8_in, 45);
    
    chassis.pid_wait_until(-4_in);
    
    actuate_back_mogo(true);
    
    chassis.pid_wait();

    IntakeInputState = 0;

  chassis.pid_turn_set(85, 80);
  chassis.pid_wait();

  IntakeInputState = 1;

    chassis.pid_odom_set(26, 75, true);
    chassis.pid_wait();

    pros::delay(500);

    chassis.pid_odom_set(-6, 80, true);
    chassis.pid_wait();

    chassis.pid_turn_set(0, 80);
    chassis.pid_wait();
    pros::delay(1000);

    actuate_back_mogo(false);

    chassis.pid_turn_set(180, 90, true);
    chassis.pid_wait();

    chassis.pid_odom_set(-9, 80, 90);
    chassis.pid_wait();


    // chassis.pid_odom_set(35, 70);
    // chassis.pid_wait_until(12);
    // nextState(3);
    chassis.pid_wait();
}

  void state_elim_goal_red_with_center_ring_no_final_ring() { 
  
    chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);
    
    chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
    
    ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue
    
    chassis.slew_odom_reenable(true);
  
    // pros::delay(1000);
    
    chassis.pid_odom_set(5.55_in, 80);
    chassis.pid_wait();
    
    
    nextState(2);
    pros::delay(500);
    nextState(0);
    
    chassis.pid_odom_set(-9_in, 100);
    chassis.pid_wait();
    
    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    
    // chassis.pid_turn_set(-90, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(-62.5, 90, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  
    // chassis.pid_odom_set(4_in, 85);
    // chassis.pid_wait();
  
  
  
    chassis.pid_odom_set(11, 80, true);
    chassis.pid_wait_until(4);
    IntakeInputState = 5;
  
    chassis.pid_wait_until(9);
  
    actuate_intake(true);
  
  
    chassis.pid_wait();
  
  
    pros::delay(125);
  
    chassis.pid_odom_set(-6, 70, true);
    chassis.pid_wait();
  
  
  
    // chassis.pid_odom_set(2_in, 80);
    // chassis.pid_wait();
  
    // actuate_right_doinker(true);
    // actuate_intake(true);
    // pros::delay(400);
  
    // chassis.pid_odom_set(-12, 70, true);
    // chassis.pid_wait();
  
    // actuate_right_doinker(false);
    // chassis.pid_wait();
  
    // chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    // chassis.pid_turn_set(-75, 80, true);
    // chassis.pid_wait();
    
    // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
  
    // IntakeInputState = 5;
  
    // chassis.pid_odom_set(10, 70, true);
    // chassis.pid_wait();
  
    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    // chassis.pid_turn_set(-150, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(-150, 80, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants
    
    
    chassis.pid_odom_set(-19_in, 70, true);
    chassis.pid_wait_quick_chain();
    chassis.pid_odom_set(-8_in, 45);
    
    chassis.pid_wait_until(-4_in);
    
    actuate_back_mogo(true);
    
    chassis.pid_wait();
  
    IntakeInputState = 4;
    
    chassis.pid_turn_set(-39, TURN_SPEED);
    chassis.pid_wait();
    
    chassis.pid_drive_set(18.5, 90, true);
    actuate_intake(false);
    IntakeInputState = 0;
    chassis.pid_wait_until(6);
    
    // chassis.pid_wait_until(3);
    
    
    chassis.pid_wait();
    actuate_right_doinker(true);
    
    pros::delay(250);
    
    // chassis.pid_turn_set(-22, TURN_SPEED);
    // chassis.pid_wait();
    
    chassis.pid_swing_exit_condition_set(50_ms, 4_deg, 125_ms, 8_deg, 250_ms, 200_ms);
    
    
    chassis.pid_swing_set(ez::LEFT_SWING, -13.5_deg, 40);
    chassis.pid_wait(); 
    
    chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
    
    
    chassis.pid_drive_set(2.75, 60, true);
    chassis.pid_wait_until(1.25);
    actuate_left_doinker(true);
    
    chassis.pid_wait();
    
    pros::delay(150);
    
    
    
    chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
    chassis.pid_wait();
    
    chassis.pid_drive_exit_condition_set(90_ms, 4_in, 250_ms, 7_in, 250_ms, 250_ms);
    
    chassis.pid_drive_set(-43, 110, true);
    actuate_intake(true);
    
    
    
    chassis.pid_wait_until(-28);
    actuate_left_doinker(false);
    
    chassis.pid_wait_until(-29);
    actuate_right_doinker(false);
    
    chassis.pid_wait();
    
    chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
    
    
    chassis.pid_turn_set(-60, TURN_SPEED);
    actuate_intake(true);
    chassis.pid_wait();
    
    chassis.pid_odom_set(9, 110, false);
    IntakeInputState = 1;
    chassis.pid_wait();

    chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants
    
    
    // chassis.pid_turn_set(-90, 80, true);
    // chassis.pid_wait();
    
    chassis.pid_turn_set(15, 90, true);
    chassis.pid_wait();
  
    chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants

  
    
    
    // chassis.pid_swing_set(ez::LEFT_SWING, 75, 75);
    
    chassis.pid_wait();
  
    chassis.pid_odom_set(5, 110, false);
    chassis.pid_wait();
  
    chassis.pid_swing_set(ez::LEFT_SWING, 90, 70);
    chassis.pid_wait();
  
    
    // chassis.pid_drive_set(14, 85, true);
    // chassis.pid_wait();
  
    // chassis.pid_turn_set(90, 80, true);
    // chassis.pid_wait();
  
    // chassis.pid_odom_set(10, 100, true);
    // chassis.pid_wait();
    
    // pros::delay(250);
  
    chassis.pid_turn_set(112, 100);
    chassis.pid_wait();
    
    
    chassis.pid_odom_set(-24, 120, true);
    chassis.pid_wait_until(-4);
    // nextState(8);
    nextState(7);
  
    LadyBrownMech.move_relative(800, 100);
    chassis.pid_wait();
    
    
    // chassis.pid_turn_set(0, 100);
    // chassis.pid_wait();
    
    
    
    // chassis.pid_drive_set(7, 110, true);
    // chassis.pid_wait();
  
    
    
    }

void state_center_grab_goal_red() {
  chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);

  chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
  
  ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue

  chassis.slew_odom_reenable(true);

  chassis.pid_odom_set(5.55_in, 80);
  chassis.pid_wait();


  nextState(2);
  pros::delay(650);
  nextState(0);

  chassis.pid_odom_set(-6_in, 85);
  chassis.pid_wait();

  // chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants


  chassis.pid_turn_set(-158, 80, true);
  chassis.pid_wait();

  // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants


  chassis.pid_odom_set(-15_in, 70, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-20_in, 45);

  chassis.pid_wait_until(-14_in);
  
  actuate_back_mogo(true);

  chassis.pid_wait();

  


  chassis.pid_turn_set(-53, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(15, 90, true);
  chassis.pid_wait_until(6);

  // chassis.pid_wait_until(3);
  

  chassis.pid_wait();
  actuate_right_doinker(true);

  pros::delay(250);

  // chassis.pid_turn_set(-22, TURN_SPEED);
  // chassis.pid_wait();
  
  chassis.pid_swing_set(ez::LEFT_SWING, -22_deg, 40);
  chassis.pid_wait();


  chassis.pid_drive_set(1.5, 60, true);
  chassis.pid_wait_until(0.75);
  actuate_left_doinker(true);

  chassis.pid_wait();

  pros::delay(150);

  chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(-45, 110, true);
  actuate_intake(true);


  
  chassis.pid_wait_until(-30);
  actuate_left_doinker(false);

  chassis.pid_wait_until(-36);
  actuate_right_doinker(false);

  chassis.pid_wait();

  chassis.pid_turn_set(-80, TURN_SPEED);
  actuate_intake(true);
  chassis.pid_wait();

  chassis.pid_drive_set(7, 110, false);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 80, 70);
  IntakeInputState = 1;

  actuate_intake(true);

  chassis.pid_wait();

  chassis.pid_drive_set(15, 100, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-65, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(19, 90, true);
  chassis.pid_wait_until(5);
  nextState(5);
  chassis.pid_wait();

  chassis.drive_set(0, 0);


}

void state_center_grab_goal_blue() {
  chassis.odom_xyt_set(-11.8_in, -55_in, -237.25_deg);

  chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
  
  ColorSorterToggle = 1; // 1 = throw out red, 2 = throw out blue

  chassis.slew_odom_reenable(true);

  chassis.pid_odom_set(5.55_in, 80);
  chassis.pid_wait();


  nextState(2);
  pros::delay(650);
  nextState(0);

  chassis.pid_odom_set(-6_in, 85);
  chassis.pid_wait();

  // chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants


  chassis.pid_turn_set(158, 80, true);
  chassis.pid_wait();

  // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants


  chassis.pid_odom_set(-15_in, 70, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-20_in, 45);

  chassis.pid_wait_until(-14_in);
  
  actuate_back_mogo(true);

  chassis.pid_wait();

  


  chassis.pid_turn_set(53, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(15, 90, true);
  chassis.pid_wait_until(6);

  // chassis.pid_wait_until(3);
  

  chassis.pid_wait();
  actuate_left_doinker(true);

  pros::delay(250);

  // chassis.pid_turn_set(-22, TURN_SPEED);
  // chassis.pid_wait();
  
  chassis.pid_swing_set(ez::RIGHT_SWING, 22_deg, 40);
  chassis.pid_wait();


  chassis.pid_drive_set(1.5, 60, true);
  chassis.pid_wait_until(0.75);
  actuate_left_doinker(true);

  chassis.pid_wait();

  pros::delay(150);

  chassis.pid_swing_set(ez::RIGHT_SWING, 40_deg, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(-45, 110, true);
  actuate_intake(true);


  
  chassis.pid_wait_until(-30);
  actuate_right_doinker(false);

  chassis.pid_wait_until(-36);
  actuate_left_doinker(false);

  chassis.pid_wait();

  chassis.pid_turn_set(80, TURN_SPEED);
  actuate_intake(true);
  chassis.pid_wait();

  chassis.pid_drive_set(7, 110, false);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -80, 70);
  IntakeInputState = 1;

  actuate_intake(true);

  chassis.pid_wait();

  chassis.pid_drive_set(15, 100, true);
  chassis.pid_wait();

  chassis.pid_turn_set(65, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(19, 90, true);
  chassis.pid_wait_until(5);
  nextState(5);
  chassis.pid_wait();

  chassis.drive_set(0, 0);


}


void state_elim_goal_red_no_alliance_stake() {
  chassis.odom_xyt_set(11.8_in, -55_in, 237.25_deg);

  chassis.pid_turn_exit_condition_set(80_ms, 3.5_deg, 150_ms, 8.5_deg, 500_ms, 500_ms);
  
  ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue

  chassis.slew_odom_reenable(true);

  chassis.pid_odom_set(-1.45_in, 85);
  chassis.pid_wait();

  // chassis.pid_odom_set(5.55_in, 80);
  // chassis.pid_wait();


  // nextState(2);
  // pros::delay(650);
  // nextState(0);

  // chassis.pid_odom_set(-6_in, 85);
  // chassis.pid_wait();

  // chassis.pid_turn_constants_set(3.1, 0.05, 25.0, 15.0);     // Turn in place constants


  chassis.pid_turn_set(-158, 80, true);
  chassis.pid_wait();

  // chassis.pid_turn_constants_set(2.9, 0.05, 24.0, 15.0);     // Turn in place constants


  chassis.pid_odom_set(-15_in, 70, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-20_in, 40);

  chassis.pid_wait_until(-14_in);
  
  actuate_back_mogo(true);

  chassis.pid_wait();

  


  chassis.pid_turn_set(-53, 80);
  IntakeInputState = 1;
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(15, 90, true);
  chassis.pid_wait_until(6);

  // chassis.pid_wait_until(3);
  

  chassis.pid_wait();
  actuate_right_doinker(true);

  pros::delay(250);

  // chassis.pid_turn_set(-22, TURN_SPEED);
  // chassis.pid_wait();
  
  chassis.pid_swing_set(ez::LEFT_SWING, -22_deg, 40);
  chassis.pid_wait();


  chassis.pid_drive_set(4, 60, true);
  chassis.pid_wait_until(3);
  actuate_left_doinker(true);

  chassis.pid_wait();

  pros::delay(150);

  chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(-40, 110, true);
  actuate_intake(true);


  
  chassis.pid_wait_until(-30);
  actuate_left_doinker(false);

  chassis.pid_wait_until(-36);
  actuate_right_doinker(false);

  chassis.pid_wait();

  chassis.pid_turn_set(-80, TURN_SPEED);
  actuate_intake(true);
  chassis.pid_wait();

  chassis.pid_drive_set(4, 110, false);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 90, 85);
  IntakeInputState = 1;

  actuate_intake(true);

  chassis.pid_wait();

  chassis.pid_drive_set(12, 100, true);
  chassis.pid_wait();

  chassis.pid_turn_set(154, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(26, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(15, 60);
  chassis.pid_wait_until(7.5);
  
  IntakeInputState = 2;
  chassis.pid_wait_until(9);
  IntakeInputState = 1;
  
  chassis.pid_wait();

  chassis.pid_drive_set(-6, 100, true);
  chassis.pid_wait();

  chassis.pid_turn_set(0, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(30, 100, true);
  chassis.pid_wait_until(20);
  actuate_back_mogo(false);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 100, true);
  chassis.pid_wait();



}

// void state_full_awp_red() {
//   chassis.odom_xyt_set(-11.8_in, -55_in, -237.25_deg);
  
//   ColorSorterToggle = 2; // 1 = throw out red, 2 = throw out blue

//   chassis.slew_odom_reenable(true);

//   chassis.pid_odom_set(5.55_in, 80);
//   chassis.pid_wait();

//   actuate_intake(true);

//   nextState(2);
//   pros::delay(900);
//   nextState(0);

//   chassis.pid_odom_set(-6_in, 80);
//   chassis.pid_wait();

  // chassis.pid_odom_set(
  //   {{-24_in, -24_in}, rev, 70});

  // chassis.pid_wait_until(28_in);
  // actuate_back_mogo(true);
  // chassis.pid_wait();

  // IntakeInputState = 1;


  // chassis.pid_odom_set({{-40_in, -7_in, -45_deg}, fwd, 70});
  // chassis.pid_wait();



  // chassis.pid_odom_set({{{-26_in, -24_in}, rev, 70},
  //   {{-48_in, -24_in}, fwd, 70}});

  //   chassis.pid_wait();
    
  //   chassis.pid_odom_set({{-12_in, -36_in}, rev, 70});
  // chassis.pid_wait();

  // chassis.pid_odom_set({{{0_in, -38_in}, fwd, 70},
  //   {{10_in, -38_in}, fwd, 70}});

  //   chassis.pid_wait_until({0_in, -44_in});
  //   actuate_back_mogo(false);

  // chassis.pid_wait();

  // chassis.

  // IntakeInputState = 5; 

  // chassis.pid_odom_set({{24_in, -24_in}, rev, 70});

  //   chassis.pid_wait_until(24.41_in);
  //   actuate_back_mogo(true);

  //   chassis.pid_wait();

  //   IntakeInputState = 1;

  //   chassis.pid_odom_set({{{48_in, -24_in}, fwd, 70},
  //     {{24_in, -20_in}, fwd, 70},
  //   {{16_in, -16_in}, fwd, 70}});

  //   chassis.pid_wait_until({24_in, -20_in});
  //   chassis.pid_wait();

  //   nextState(2);


  //   chassis.drive_set(0, 0);

// }

void od_new_red_right_center_ring_grab() {
 chassis.odom_xyt_set(12_in, -60_in, 219.10_deg);
 chassis.slew_odom_reenable(true);

 nextState(2);
 pros::delay(1500);

 nextState(0);

 chassis.pid_swing_set(ez::RIGHT_SWING, -80_deg, 90);
 chassis.pid_wait();

 chassis.pid_odom_set({{2_in, -48_in}, fwd, 60});
 IntakeInputState = 5;
 chassis.pid_wait_until(6_in);
 chassis.pid_wait();

 actuate_intake(true);

 pros::delay(450);

 chassis.pid_odom_set({{24_in, -24_in}, rev, 80});
 chassis.pid_wait_until(-26_in);

 actuate_back_mogo(true);

 chassis.pid_wait();

 IntakeInputState = 1;

 chassis.pid_odom_set({{8_in, -8_in, -60_deg}, fwd, 80});
 chassis.pid_wait_until(8_in);
 IntakeInputState = 0;
 chassis.pid_wait();

 actuate_right_doinker(true);

 chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
 chassis.pid_wait();

 actuate_left_doinker(true);

 chassis.pid_odom_set({{36_in, -36_in}, rev, 80});
 chassis.pid_wait();

 chassis.pid_turn_set(-40, 70);
 chassis.pid_wait();

 IntakeInputState = 1;

 chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 60);
 chassis.pid_wait();

 chassis.pid_odom_set({{46_in, -24_in}, fwd, 80});
 chassis.pid_wait();

 chassis.pid_odom_set({{{50_in, -50_in}, fwd, 80},
  {{56_in, -56_in}, fwd, 80},
  {{60_in, -60_in}, fwd, 80}});
 
  chassis.pid_wait_until({50_in, -50_in});
  IntakeInputState = 2;

  chassis.pid_wait_until({56_in, -56_in});
  IntakeInputState = 1;
  chassis.pid_wait();

  chassis.pid_odom_set({{26_in, -30_in}, rev, 80});
  chassis.pid_wait();

  chassis.pid_turn_set(28, 70);
  chassis.pid_wait();

  actuate_back_mogo(false);

  chassis.pid_turn_set(170, 70);
  chassis.pid_wait();



 


}

void od_red_right_center_ring_grab_with_alliance_stake() {
  chassis.odom_theta_set(-60);

  // first ring stack

  chassis.pid_odom_set(15_in, 45, true);
  // chassis.pid_wait_until(5);


  Intake.move_relative(100000, 600);

  // RingLift.move_relative(100000, 300);

  // pros::delay(815);

  // pros::delay(700);


  chassis.pid_wait_until(13);

  actuate_intake(true);

  // Intake.move_relative(100000, 600);


  chassis.pid_wait();

  stop_intake_auto();

  chassis.pid_turn_set(0, 55);
  
  chassis.pid_wait();

  pros::delay(250); // temporary

  // score first ring on alliance stake
  chassis.pid_odom_set(-9.5, 85, true);

  chassis.pid_wait_until(-6);

  actuate_intake(true);
  // actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();

  pros::delay(260);

  chassis.pid_odom_set(13.5, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-135, TURN_SPEED);
  pros::delay(200);
  stop_intake_auto();
  chassis.pid_wait();

  chassis.pid_odom_set(-34, 90, true);
  chassis.pid_wait_until(-30);
  actuate_back_mogo(true);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  chassis.pid_turn_set(-56, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(17, 90, true);
  chassis.pid_wait_until(6);
  stop_intake_auto();
  actuate_intake(false);
  chassis.pid_wait();

  actuate_right_doinker(true);
  actuate_right_doinker(true);


  pros::delay(250);
  
  chassis.pid_swing_set(ez::LEFT_SWING, -26_deg, 50);
  chassis.pid_wait();

  chassis.pid_odom_set(1.5, 60, true);
  chassis.pid_wait();

  actuate_left_doinker(true);
  actuate_left_doinker(false);

  pros::delay(150);

  chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(-38, DRIVE_SPEED, true);

  
  chassis.pid_wait_until(-26);
  actuate_right_doinker(false);

  chassis.pid_wait_until(-30);
  actuate_left_doinker(true);

  chassis.pid_wait();

  chassis.pid_turn_set(-80, TURN_SPEED);
  actuate_intake(true);
  chassis.pid_wait();

  chassis.pid_odom_set(4, DRIVE_SPEED, false);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 90, 60);
  spin_intake_auto(true, 600);

  chassis.pid_wait();

  chassis.pid_odom_set(14, 100, true);
  chassis.pid_wait();

  pros::delay(2000);

  actuate_back_mogo(false);

  chassis.pid_odom_set(4, 100, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 90);
  chassis.pid_wait();
}

void blue_left_solo_awp() {
  chassis.odom_theta_set(60);

  // first ring stack

  chassis.pid_drive_set(15_in, 45, true);
  // chassis.pid_wait_until(5);


  Intake.move_relative(100000, 600);

  // RingLift.move_relative(100000, 300);

  // pros::delay(815);

  // pros::delay(700);


  chassis.pid_wait_until(13);

  actuate_intake(true);

  // Intake.move_relative(100000, 600);


  chassis.pid_wait();

  stop_intake_auto();

  chassis.pid_turn_set(0, 55);
  
  chassis.pid_wait();

  // pros::delay(250); // temporary

  // score first ring on alliance stake
  chassis.pid_drive_set(-10, 85, true);

  chassis.pid_wait_until(-6);

  actuate_intake(true);
  // actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();

  pros::delay(250);

  chassis.pid_drive_set(14, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_set(135, TURN_SPEED);
  pros::delay(200);
  stop_intake_auto();
  chassis.pid_wait();

  chassis.pid_drive_set(-34, 75, true);
  chassis.pid_wait_until(-30);
  actuate_back_mogo(true);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  chassis.pid_turn_set(-100, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(16, 70, true);
  chassis.pid_wait();

  pros::delay(1750);

  chassis.pid_drive_set(-6, 70, true);
  chassis.pid_wait();

  chassis.pid_turn_set(80, TURN_SPEED);

  chassis.pid_wait();

  LadyBrownMech.move_relative(1500, 600);
  pros::delay(300);

  actuate_back_mogo(false);

  stop_intake_auto();

  chassis.pid_drive_set(14, 60);
  chassis.pid_wait();
  


}

void red_right_solo_awp() {
  chassis.odom_theta_set(-60);

  // first ring stack

  chassis.pid_drive_set(15_in, 45, true);
  // chassis.pid_wait_until(5);


  Intake.move_relative(100000, 600);

  // RingLift.move_relative(100000, 300);

  // pros::delay(815);

  // pros::delay(700);


  chassis.pid_wait_until(13);

  actuate_intake(true);

  // Intake.move_relative(100000, 600);


  chassis.pid_wait();

  stop_intake_auto();

  chassis.pid_turn_set(0, 55);
  
  chassis.pid_wait();

  // pros::delay(250); // temporary

  // score first ring on alliance stake
  chassis.pid_drive_set(-10, 85, true);

  chassis.pid_wait_until(-6);

  actuate_intake(true);
  // actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();

  pros::delay(250);

  chassis.pid_drive_set(14, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-135, TURN_SPEED);
  pros::delay(200);
  stop_intake_auto();
  chassis.pid_wait();

  chassis.pid_drive_set(-34, 75, true);
  chassis.pid_wait_until(-30);
  actuate_back_mogo(true);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  chassis.pid_turn_set(100, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(16, 70, true);
  chassis.pid_wait();

  pros::delay(1750);

  chassis.pid_turn_set(-50, TURN_SPEED);

  chassis.pid_wait();

  LadyBrownMech.move_relative(1500, 600);
  pros::delay(300);

  actuate_back_mogo(false);

  stop_intake_auto();

  chassis.pid_drive_set(18, 100);
  chassis.pid_wait();
  


}

void blue_left_center_ring_grab() {
  chassis.odom_theta_set(60);

  // first ring stack

  chassis.pid_drive_set(15_in, 45, true);
  // chassis.pid_wait_until(5);


  Intake.move_relative(100000, 600);

  // RingLift.move_relative(100000, 300);

  // pros::delay(815);

  // pros::delay(700);


  chassis.pid_wait_until(13);

  actuate_intake(true);

  // Intake.move_relative(100000, 600);


  chassis.pid_wait();

  stop_intake_auto();

  chassis.pid_turn_set(0, 55);
  
  chassis.pid_wait();

  // pros::delay(250); // temporary

  // score first ring on alliance stake
  chassis.pid_drive_set(-10, 85, true);

  chassis.pid_wait_until(-6);

  actuate_intake(true);
  // actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();

  pros::delay(220);

  chassis.pid_drive_set(14, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_set(135, TURN_SPEED);
  pros::delay(200);
  stop_intake_auto();
  chassis.pid_wait();

  chassis.pid_drive_set(-34, 90, true);
  chassis.pid_wait_until(-30);
  actuate_back_mogo(true);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  chassis.pid_turn_set(53, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(16, 90, true);
  chassis.pid_wait_until(6);
  stop_intake_auto();
  actuate_intake(false);
  chassis.pid_wait();

  actuate_left_doinker(true);

  pros::delay(250);
  
  chassis.pid_swing_set(ez::RIGHT_SWING, -18_deg, 50);
  chassis.pid_wait();

  chassis.pid_drive_set(1.5, 60, true);
  chassis.pid_wait();

  actuate_right_doinker(true);

  pros::delay(150);

  chassis.pid_swing_set(ez::RIGHT_SWING, 40_deg, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(-38, DRIVE_SPEED, true);

  
  chassis.pid_wait_until(-26);
  actuate_right_doinker(false);

  chassis.pid_wait_until(-30);
  actuate_left_doinker(false);

  chassis.pid_wait();

  chassis.pid_turn_set(80, TURN_SPEED);
  actuate_intake(true);
  chassis.pid_wait();

  chassis.pid_drive_set(4, DRIVE_SPEED, false);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -90, 60);
  spin_intake_auto(true, 600);

  chassis.pid_wait();

  chassis.pid_drive_set(14, 100, true);
  chassis.pid_wait();

  chassis.pid_turn_set(10, TURN_SPEED);
  chassis.pid_wait();

  pros::delay(550);

  // chassis.pid_drive_set(-9, 100, true);
  // chassis.pid_wait();

  actuate_back_mogo(false);

  stop_intake_auto();

  chassis.pid_drive_set(13, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait();

}

void red_right_center_ring_grab() {
  chassis.odom_theta_set(-60);

  // first ring stack

  chassis.pid_drive_set(15_in, 45, true);
  // chassis.pid_wait_until(5);


  Intake.move_relative(100000, 600);

  // RingLift.move_relative(100000, 300);

  // pros::delay(815);

  // pros::delay(700);


  chassis.pid_wait_until(13);

  actuate_intake(true);

  // Intake.move_relative(100000, 600);


  chassis.pid_wait();

  stop_intake_auto();

  chassis.pid_turn_set(0, 55);
  
  chassis.pid_wait();

  pros::delay(250); // temporary

  // score first ring on alliance stake
  chassis.pid_drive_set(-10, 85, true);

  chassis.pid_wait_until(-6);

  actuate_intake(true);
  // actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();

  pros::delay(220);

  chassis.pid_drive_set(14, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-135, TURN_SPEED);
  pros::delay(200);
  stop_intake_auto();
  chassis.pid_wait();

  chassis.pid_drive_set(-34, 90, true);
  chassis.pid_wait_until(-30);
  actuate_back_mogo(true);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  chassis.pid_turn_set(-53, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(16, 90, true);
  chassis.pid_wait_until(6);
  stop_intake_auto();
  actuate_intake(false);
  chassis.pid_wait();

  actuate_right_doinker(true);

  pros::delay(250);
  
  chassis.pid_swing_set(ez::RIGHT_SWING, 18_deg, 50);
  chassis.pid_wait();

  chassis.pid_drive_set(1.5, 60, true);
  chassis.pid_wait();

  actuate_left_doinker(true);

  pros::delay(150);

  chassis.pid_swing_set(ez::LEFT_SWING, -40_deg, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(-38, DRIVE_SPEED, true);

  
  chassis.pid_wait_until(-26);
  actuate_left_doinker(false);

  chassis.pid_wait_until(-30);
  actuate_right_doinker(false);

  chassis.pid_wait();

  chassis.pid_turn_set(-80, TURN_SPEED);
  actuate_intake(true);
  chassis.pid_wait();

  chassis.pid_drive_set(4, DRIVE_SPEED, false);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 90, 60);
  spin_intake_auto(true, 600);

  chassis.pid_wait();

  chassis.pid_drive_set(14, 100, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-10, TURN_SPEED);
  chassis.pid_wait();

  pros::delay(550);

  // chassis.pid_drive_set(-9, 100, true);
  // chassis.pid_wait();

  actuate_back_mogo(false);

  stop_intake_auto();

  chassis.pid_drive_set(13, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait();

}

void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  // chassis.odom_xyt_set(-60_in, -11_in, 100_deg);

  

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_swing_set(ez::LEFT_SWING, 60_deg, 70);
  // chassis.pid_wait();

  // chassis.pid_odom_set({{24_in, 0_in}, fwd, 70});

  chassis.pid_odom_pp_set({{{24_in, 0_in}, fwd, }, {{48_in, 48_in}, fwd, 70}});

  chassis.pid_wait();

  // chassis.pid_swing_set(ez::RIGHT_SWING, -60_deg, 70);
  // chassis.pid_wait();

  pros::delay(2000);

  // spin_intake_auto(true, 600);
  // pros::delay(3000);
  // stop_intake_auto();

  // chassis.pid_turn_set(90_deg, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_turn_set(180_deg, TURN_SPEED);
  // chassis.pid_wait();

  // pros::delay(13500);

  
  // chassis.pid_turn_set(90, TURN_SPEED);

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, false, true);
  pros::delay(400);

  // driveStraightPID(100, 0,120);

  // chassis.pid_wait();
  // chassis.drive_set(0, 0);
  // pros::delay(3000);
  


  // chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // chassis.drive_set(0, 0);

  // chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();
}

void left_elim() {
  chassis.odom_theta_set(57.2);

  // first ring stack

  chassis.pid_drive_set(17_in, 70, true);
  chassis.pid_wait_until(5);

  // actuate_intake(false);

  Intake.move_relative(100000, 600);
  // RingLift.move_relative(100000, 260);

  pros::delay(700);

  // chassis.pid_wait_until(11);

  actuate_intake(false);

  stop_intake_auto();

  chassis.pid_wait();

  // chassis.pid_drive_set(-3_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // stop_intake_auto();
  // spin_intake_auto(true, 600, true);

  // pros::delay(1000);

  // towards alliance stake

  chassis.pid_turn_set(0, 60);
  
  chassis.pid_wait();

  pros::delay(250); // temporary

  // score first ring on alliance stake
  chassis.pid_drive_set(-9.5, 70, true);

  chassis.pid_wait_until(-2);

  actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();


  pros::delay(750);

  // stop_intake_auto();

  // turn towards mogo

  chassis.pid_drive_set(6.5, 55, true);
  chassis.pid_wait();

  pros::delay(350);


  chassis.pid_turn_set(138, TURN_SPEED);
  pros::delay(250);
  stop_intake_auto();
  chassis.pid_wait();

  // grab mogo

  chassis.pid_drive_set(-22, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  // pros::delay(500);
  chassis.pid_drive_set(-20, 40);

  // pros::delay(150);

  chassis.pid_wait_until(-11);

  actuate_back_mogo(true);

  chassis.pid_wait();

  pros::delay(200);


  chassis.pid_turn_set(-105, TURN_SPEED);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  chassis.pid_drive_set(4, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(11, 50 );

  pros::delay(2000);

  stop_intake_auto();
}

void skillz2() {

  chassis.odom_theta_set(0);
  
  actuate_intake(false);

  // score first ring on alliance stake

  spin_intake_auto(true, 600);

  pros::delay(400);

  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  stop_intake_auto();
  chassis.pid_wait();

  chassis.pid_turn_set(-90, TURN_SPEED);
  chassis.pid_wait();

  // pick up mogo #1

  chassis.pid_drive_set(-11, 80, true);
  chassis.pid_wait_quick_chain();
  // pros::delay(500);
  chassis.pid_drive_set(-7, 20);

  // pros::delay(150);

  chassis.pid_wait_until(-5.5);
  actuate_back_mogo(true);
  chassis.pid_wait();
  pros::delay(100);

  chassis.pid_drive_set(-3, DRIVE_SPEED);
  chassis.pid_wait();

  // pick up first ring

  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  chassis.pid_drive_set(13, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10, 40);
  chassis.pid_wait();

  pros::delay(500);

  // pick up second ring

  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(18, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10, 45);
  chassis.pid_wait();

  pros::delay(750);

  chassis.pid_drive_set(-8, DRIVE_SPEED);
  chassis.pid_wait();

  // towards center

  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(23.5, DRIVE_SPEED);
  chassis.pid_wait();

  // turn towards wall stake ring

  chassis.pid_turn_set(90, TURN_SPEED);
  // nextState();
  chassis.pid_wait();

  // score wall stake

  chassis.pid_drive_set(3, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(6, 30);
  chassis.pid_wait();

  // pros::delay(1500);

  // spin_intake_auto(false, 600);
  // pros::delay(50);
  // stop_intake_auto();f
  // nextState();
  // nextState();

  pros::delay(750);

  chassis.pid_drive_set(-10, 45);
  // chassis.pid_wait_until(-4);
  // nextState();
  chassis.pid_wait();

  // towards two rings

  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait();

  // pick up two rings

  chassis.pid_drive_set(35, DRIVE_SPEED);
  spin_intake_auto(true, 600);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(26, 30);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-22, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(130, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(12, 40);
  chassis.pid_wait();

  pros::delay(800);

  chassis.pid_drive_set(-16,DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-40, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-27, DRIVE_SPEED);
  actuate_back_mogo(false);
  chassis.pid_wait();


  chassis.pid_drive_set(10, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-59, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12, 20);
  chassis.pid_wait_until(-2);
  actuate_back_mogo(true);
  chassis.pid_wait();

///////////////////////
  // left side
/////////////////////////////////////

  // pick up first ring

  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  chassis.pid_drive_set(13, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10, 40);
  chassis.pid_wait();

  pros::delay(500);

  // pick up second ring

  chassis.pid_turn_set(-90, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(18, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10, 45);
  chassis.pid_wait();

  pros::delay(750);

  chassis.pid_drive_set(-4.5, DRIVE_SPEED);
  chassis.pid_wait();

  // towards center

  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(21, DRIVE_SPEED);
  chassis.pid_wait();

  // turn towards wall stake ring

  chassis.pid_turn_set(-90, TURN_SPEED);
  nextState();
  chassis.pid_wait();

  // score wall stake

  chassis.pid_drive_set(6, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(9, 30);
  chassis.pid_wait();

  pros::delay(1500);

  spin_intake_auto(false, 600);
  pros::delay(50);
  stop_intake_auto();
  nextState();
  nextState();

  pros::delay(750);

  chassis.pid_drive_set(-11.5, 45);
  chassis.pid_wait_until(-4);
  nextState();
  chassis.pid_wait();

  // towards two rings

  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait();

  // pick up two rings

  chassis.pid_drive_set(34, DRIVE_SPEED);
  spin_intake_auto(true, 600);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(26, 45);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-22, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-130, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(12, 40);
  chassis.pid_wait();

  pros::delay(600);

  chassis.pid_drive_set(-16,DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(25, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-16, DRIVE_SPEED);
  actuate_back_mogo(false);
  chassis.pid_wait();


  chassis.pid_drive_set(4, DRIVE_SPEED);
  chassis.pid_wait();






  
  nextState();


  


}

void safe_left() {
  chassis.odom_theta_set(57.2);

  // first ring stack

  chassis.pid_drive_set(17_in, 70, true);
  chassis.pid_wait_until(5);

  // actuate_intake(false);

  Intake.move_relative(100000, 600);
  // RingLift.move_relative(100000, 260);

  pros::delay(700);

  // chassis.pid_wait_until(11);

  actuate_intake(false);

  stop_intake_auto();

  chassis.pid_wait();

  // chassis.pid_drive_set(-3_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // stop_intake_auto();
  // spin_intake_auto(true, 600, true);

  // pros::delay(1000);

  // towards alliance stake

  chassis.pid_turn_set(0, 60);
  
  chassis.pid_wait();

  pros::delay(250); // temporary

  // score first ring on alliance stake
  chassis.pid_drive_set(-11, 70, true);

  chassis.pid_wait_until(-3);

  actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();


  pros::delay(750);

  // stop_intake_auto();

  // turn towards mogo

  chassis.pid_drive_set(7, 55, true);
  chassis.pid_wait();

  pros::delay(350);


  chassis.pid_turn_set(138, TURN_SPEED);
  pros::delay(250);
  stop_intake_auto();
  chassis.pid_wait();

  // grab mogo

  chassis.pid_drive_set(-22, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  // pros::delay(500);
  chassis.pid_drive_set(-20, 50);

  // pros::delay(150);

  chassis.pid_wait_until(-11);

  actuate_back_mogo(true);

  chassis.pid_wait();

  pros::delay(200);


  chassis.pid_turn_set(-105, TURN_SPEED);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  chassis.pid_drive_set(4, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(11, 50 );

  pros::delay(2000);

  spin_intake_auto(false, 600);

  chassis.pid_turn_set(-150, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(30, 70, true);
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);

  chassis.pid_wait();

  stop_intake_auto();

  // LadyBrownMech.set_brake_mode(pros::MotorBrake::hold);

  // LadyBrownMech.move_relative(375, 600);
  // pros::delay(300);

  // chassis.pid_drive_set(-31.5_in, 70, true);

  // // chassis.pid_wait_until(-15);
  // stop_intake_auto();
  // chassis.pid_wait();

  // LadyBrownMech.set_brake_mode(pros::MotorBrake::coast);
  // chassis.drive_set(0,0);

  // chassis.pid_turn_set(105, TURN_SPEED);
  // chassis.pid_wait();


  // chassis.pid_drive_set(48, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // stop_intake_auto();

  // actuate_back_mogo(false);

  // chassis.pid_turn_set(-125, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_drive_set(-22, DRIVE_SPEED, true);
  // chassis.pid_wait_quick_chain();
  // // pros::delay(500)
  // chassis.pid_drive_set(-18, 50, true);

  // // pros::delay(150);

  // chassis.pid_wait_until(-10.5);

  // actuate_back_mogo(true);

  // chassis.pid_wait();

  // // pros::delay(200);

  // chassis.pid_turn_set(95, TURN_SPEED);
  // chassis.pid_wait();

  // spin_intake_auto(true, 600);

  // chassis.pid_drive_set(16, DRIVE_SPEED, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(12, 70 );



  // LadyBrownMech.set_brake_mode(pros::MotorBrake::hold);

  // LadyBrownMech.move_relative(375, 600);
  // pros::delay(300);

  // chassis.pid_turn_set(105, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_drive_set(-46_in, DRIVE_SPEED, true);

  // chassis.pid_wait_until(-15);
  // stop_intake_auto();
  // chassis.pid_wait();

  // LadyBrownMech.set_brake_mode(pros::MotorBrake::coast);
}

void elim_auto_2() {
  chassis.odom_theta_set(57.2);

  // first ring stack

  chassis.pid_drive_set(17_in, 65, true);
  chassis.pid_wait_until(5);

  // actuate_intake(false);

  Intake.move_relative(100000, 600);
  // RingLift.move_relative(100000, 260);

  pros::delay(300);

  // chassis.pid_wait_until(11);

  actuate_intake(false);

  // stop_intake_auto();

  chassis.pid_wait();

  // chassis.pid_drive_set(-3_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // stop_intake_auto();
  // spin_intake_auto(true, 600, true);

  // pros::delay(1000);

  // towards alliance stake

  chassis.pid_turn_set(0, 60);
  
  chassis.pid_wait();

  pros::delay(250); // temporary

  // score first ring on alliance stake
  chassis.pid_drive_set(-9.5, DRIVE_SPEED, true);

  chassis.pid_wait_until(-2);

  actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();


  pros::delay(750);

  // stop_intake_auto();

  // turn towards mogo

  chassis.pid_drive_set(6.5, DRIVE_SPEED, true);
  chassis.pid_wait();

  stop_intake_auto();

  chassis.pid_turn_set(135, TURN_SPEED);
  // pros::delay(150);
  // chassis.pid_wait_until(60);
  stop_intake_auto();
  chassis.pid_wait();

  // grab mogo

  chassis.pid_drive_set(-22, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  // pros::delay(500);
  chassis.pid_drive_set(-18, 50, true);

  chassis.pid_wait_until(-6.5);

  actuate_back_mogo(true);

  pros::delay(50);

  // turn towards center rings

  chassis.pid_turn_set(-70, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(24, 50, false);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  chassis.pid_swing_set(ez::LEFT_SWING, -10_deg, 50);
  chassis.pid_wait();

  // chassis.pid_turn_set(-100, TURN_SPEED);
  // chassis.pid_wait();  

  pros::delay(300);

  chassis.pid_drive_set(4, 50, false);
  chassis.pid_wait();

  chassis.pid_drive_set(-2, 50, false);
  chassis.pid_wait();

  // chassis.pid_turn_set(-80, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_drive_set(6, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // chassis.pid_swing_set(ez::LEFT_SWING, -10_deg, 50);
  // chassis.pid_wait();


  // chassis.pid_swing_set(ez::RIGHT_SWING, -50_deg, 70);
  // chassis.pid_wait();

  // intake first ring

  // chassis.pid_drive_set(6, 65, true);
  // chassis.pid_wait();

  chassis.pid_drive_set(-3, 65, true);
  chassis.pid_wait();

  // chassis.pid_swing_set(ez::LEFT_SWING, -87_deg, 70);
  // // chassis.pid_wait();

  chassis.pid_turn_set(-70, TURN_SPEED);
  chassis.pid_wait();

  // intake second ring

  chassis.pid_drive_set(12, 40, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-80, TURN_SPEED);
  chassis.pid_wait();

  // out of center rings

  chassis.pid_drive_set(-2.5, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait();

  // towards near ring stack

  chassis.pid_drive_set(10, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(4, 90);
  chassis.pid_wait();


  // chassis.pid_turn_set(-90, TURN_SPEED);
  // chassis.pid_wait();




  // chassis.drive_set(0, 0);
}

void red_right() {
  chassis.odom_theta_set(-57.2);

  // first ring stack

  chassis.pid_drive_set(17_in, 70, true);
  chassis.pid_wait_until(5);

  // actuate_intake(false);

  Intake.move_relative(100000, 600);
  // RingLift.move_relative(100000, 300);

  // pros::delay(815);

  pros::delay(700);


  // chassis.pid_wait_until(11);

  actuate_intake(false);

  stop_intake_auto();

  chassis.pid_wait();

  // chassis.pid_drive_set(-3_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // stop_intake_auto();
  // spin_intake_auto(true, 600, true);

  // pros::delay(1000);

  // towards alliance stake

  chassis.pid_turn_set(0, 60);
  
  chassis.pid_wait();

  pros::delay(250); // temporary

  // score first ring on alliance stake
  chassis.pid_drive_set(-9.5, DRIVE_SPEED, true);

  chassis.pid_wait_until(-2);

  actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();


  pros::delay(750);

  // stop_intake_auto();

  // turn towards mogo

  chassis.pid_drive_set(6.5, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-135, TURN_SPEED);
  pros::delay(400);
  stop_intake_auto();
  chassis.pid_wait();

  

  // grab mogo

  chassis.pid_drive_set(-22, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  // pros::delay(500);
  chassis.pid_drive_set(-18, 50, true);

  chassis.pid_wait_until(-6.5);

  actuate_back_mogo(true);

  pros::delay(50);

  chassis.pid_turn_set(90, TURN_SPEED);
  
  LadyBrownMech.set_brake_mode(pros::MotorBrake::hold);

  LadyBrownMech.move_relative((375*4), 600);
  pros::delay(500);

  chassis.pid_wait();
  LadyBrownMech.move_velocity(0);

  chassis.pid_drive_set(8, DRIVE_SPEED, true);
  spin_intake_auto(true, 600);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8, 45, false);

  pros::delay(400);

  chassis.pid_turn_set(285, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(18, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45, TURN_SPEED);
  chassis.pid_wait();


  LadyBrownMech.set_brake_mode(pros::MotorBrake::coast);

  chassis.pid_drive_set(5_in, 45, true);
  chassis.pid_wait();

  pros::delay(2000);

  stop_intake_auto();


  chassis.drive_set(0,0);





  // turn towards center rings

  // chassis.pid_turn_set(30, TURN_SPEED);
  // chassis.pid_wait();

  // spin_intake_auto(true, 600);

  

  // chassis.pid_turn_set(80, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_drive_set(3, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // chassis.pid_swing_set(ez::LEFT_SWING, 10_deg, 50);
  // chassis.pid_wait();


  // // chassis.pid_swing_set(ez::RIGHT_SWING, -50_deg, 70);
  // // chassis.pid_wait();

  // // intake first ring

  // chassis.pid_drive_set(4, 65, true);
  // chassis.pid_wait();

  // chassis.pid_swing_set(ez::LEFT_SWING, 87_deg, 70);
  // chassis.pid_wait();

  // // chassis.pid_turn_set(-87, TURN_SPEED);
  // // chassis.pid_wait();

  // // intake second ring

  // chassis.pid_drive_set(12, 40, true);
  // chassis.pid_wait();

  // chassis.pid_turn_set(80, TURN_SPEED);
  // chassis.pid_wait();

  // // out of center rings

  // chassis.pid_drive_set(-2.5, 80, true);
  // chassis.pid_wait();

  // chassis.pid_turn_set(180, TURN_SPEED);
  // chassis.pid_wait();

  // // towards near ring stack

  // chassis.pid_drive_set(10, DRIVE_SPEED);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(4, 90);
  // chassis.pid_wait();


  // chassis.pid_turn_set(-90, TURN_SPEED);
  // chassis.pid_wait();




  // chassis.drive_set(0, 0);
}

void elim_auto_3() {
  chassis.odom_theta_set(57.2);

  // first ring stack

  chassis.pid_drive_set(17_in, 70, true);
  chassis.pid_wait_until(5);

  // actuate_intake(false);

  Intake.move_relative(100000, 600);
  RingLift.move_relative(100000, 260);

  pros::delay(640);

  // chassis.pid_wait_until(11);

  actuate_intake(false);

  stop_intake_auto();

  chassis.pid_wait();

  // chassis.pid_drive_set(-3_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // stop_intake_auto();
  // spin_intake_auto(true, 600, true);

  // pros::delay(1000);

  // towards alliance stake

  chassis.pid_turn_set(0, 60);
  
  chassis.pid_wait();

  pros::delay(250); // temporary

  // score first ring on alliance stake
  chassis.pid_drive_set(-9.5, DRIVE_SPEED, true);

  chassis.pid_wait_until(-2);

  actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();


  pros::delay(750);

  stop_intake_auto();

  // turn towards mogo

  chassis.pid_drive_set(6.5, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-135, TURN_SPEED);
  chassis.pid_wait();

  // grab mogo

  chassis.pid_drive_set(-22, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  // pros::delay(500);
  chassis.pid_drive_set(-18, 50, true);

  chassis.pid_wait_until(-6.5);

  actuate_back_mogo(true);

  pros::delay(50);

  // turn towards center rings

  chassis.pid_turn_set(30, TURN_SPEED);
  chassis.pid_wait();

  spin_intake_auto(true, 600);

  

  chassis.pid_turn_set(80, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(3, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 10_deg, 50);
  chassis.pid_wait();


  // chassis.pid_swing_set(ez::RIGHT_SWING, -50_deg, 70);
  // chassis.pid_wait();

  // intake first ring

  chassis.pid_drive_set(4, 65, true);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 87_deg, 70);
  chassis.pid_wait();

  // chassis.pid_turn_set(-87, TURN_SPEED);
  // chassis.pid_wait();

  // intake second ring

  chassis.pid_drive_set(12, 40, true);
  chassis.pid_wait();

  chassis.pid_turn_set(80, TURN_SPEED);
  chassis.pid_wait();

  // out of center rings

  chassis.pid_drive_set(-2.5, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait();

  // towards near ring stack

  chassis.pid_drive_set(10, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(4, 90);
  chassis.pid_wait();


  // chassis.pid_turn_set(-90, TURN_SPEED);
  // chassis.pid_wait();




  // chassis.drive_set(0, 0);
}


void skills_sig() {

  chassis.odom_theta_set(0);

  actuate_intake(true);

  spin_intake_auto(true,600,false);
  pros::delay(500);
  stop_intake_auto();

  chassis.pid_drive_set(9_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-90, TURN_SPEED);
  chassis.pid_wait();

  // grab mogo pt 1
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-10);
  actuate_back_mogo(true);
  pros::delay(500);

  // grab ring pt 1
  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();

  spin_intake_auto(true, 600,false);
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait();

  // drive forward into two rings pt 1
  spin_intake_auto(true, 600);
  chassis.pid_drive_set(28_in, 50, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-7, DRIVE_SPEED, true);
  chassis.pid_wait();

  // turn towards center pt 1
  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(32_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  // turn towards wall stake pt 1
  chassis.pid_turn_set(90,TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(9_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-7_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  // turn towards far ring pt 1
  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();

  // drive towards far ring pt 1
  spin_intake_auto(true, 600);
  chassis.pid_drive_set(17, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(500);

  // drive back pt 1
  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(60, DRIVE_SPEED, true);
  chassis.pid_wait();

  spin_intake_auto(true, 600,false);
  chassis.pid_wait();

  // drop mogo in corner pt 1
  chassis.pid_drive_set(-10, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-45, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12, DRIVE_SPEED, true);
  chassis.pid_wait();

  actuate_back_mogo(false);

  chassis.pid_drive_set(7, DRIVE_SPEED, true);
  chassis.pid_wait();

  // go to left
  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-50, DRIVE_SPEED, true);
  chassis.pid_wait();

  // grab mogo pt 2
  actuate_back_mogo(true);
  pros::delay(500);

  // grab ring pt 2
  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();

  spin_intake_auto(true, 600,false);
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-90, TURN_SPEED);
  chassis.pid_wait();

  // drive forward into two rings pt 2
  spin_intake_auto(true, 600);
  chassis.pid_drive_set(28_in, 50, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-7, DRIVE_SPEED, true);
  chassis.pid_wait();

  // turn towards center pt 2
  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(32_in, DRIVE_SPEED, true);
  chassis.pid_wait();

}

void drive_and_score() {
  chassis.odom_theta_set(180);

  actuate_intake(false);

  chassis.pid_drive_set(-22_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-20);

  actuate_back_mogo(true);

  chassis.pid_wait();

  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_wait();




  spin_intake_auto(true, 600);


  chassis.pid_turn_set(-90, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(8_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(16_in, 40, true);
  chassis.pid_wait();

  pros::delay(700);

  chassis.pid_turn_set(-110, TURN_SPEED);
  chassis.pid_wait();

  LadyBrownMech.set_brake_mode(pros::MotorBrake::hold);

  LadyBrownMech.move_relative(375, 600);
  pros::delay(300);

  chassis.pid_drive_set(-37_in, 90, true);
  chassis.pid_wait_until(-12);
  stop_intake_auto();
  chassis.pid_wait();

  LadyBrownMech.set_brake_mode(pros::MotorBrake::coast);


  chassis.drive_set(0,0);
}

void full_awp_sig() {

  chassis.odom_theta_set(48.5);

  actuate_intake(true);

  // spin_intake_auto(true, 600, true);

  // drive to ring stack
  chassis.pid_drive_set(14_in, 85, true);

  chassis.pid_wait_until(2);

  // actuate_intake(false);

  Intake.move_relative(100000, 600);
  RingLift.move_relative(100000, 400);

  pros::delay(550);

  // chassis.pid_wait_until(11);

  actuate_intake(false);

  stop_intake_auto();

  chassis.pid_wait();

  chassis.pid_drive_set(-3_in, DRIVE_SPEED);
  chassis.pid_wait();

  // stop_intake_auto();
  // spin_intake_auto(true, 600, true);

  // pros::delay(1000);

  chassis.pid_turn_set(-12, TURN_SPEED);
  
  chassis.pid_wait();

  // drive to alliance stake
  chassis.pid_drive_set(-5.75, DRIVE_SPEED, true);

  chassis.pid_wait_until(-2);

  actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();


  pros::delay(750);

  stop_intake_auto();

  chassis.drive_set(0, 0);

  // turn to mogo
  chassis.pid_drive_set(3,DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(147.5,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-14,DRIVE_SPEED, true);
  chassis.pid_wait_until(-2);
  actuate_back_mogo(true);
  chassis.pid_drive_set(-8, DRIVE_SPEED);

  chassis.pid_wait();

  chassis.pid_turn_set(-90,TURN_SPEED);
  chassis.pid_wait();

  spin_intake_auto(true, 600);
  chassis.pid_drive_set(10,DRIVE_SPEED, true);
  chassis.pid_drive_set(3,50);

  chassis.pid_wait();

  chassis.pid_turn_set(120, TURN_SPEED);
  chassis.pid_wait();


  pros::delay(500);

}

void red_positive() {

  // THE FIRST ONE IS LEFT, SECOND IS RIGHT
  
  stop_intake_auto();
  if (chassis.odom_theta_direction_get()) {
    chassis.pid_drive_set(18_in, 50, true);
    chassis.pid_wait_until(4);

  } else {
    chassis.pid_drive_set(18_in, 50, true);
    // chassis.pid_wait_until(14);
    chassis.pid_wait_until(7);

  }

  spin_intake_auto(true, 600);

  actuate_intake(false);

  pros::delay(750);

  actuate_intake(true);

  if (chassis.odom_theta_direction_get()) {
    chassis.pid_drive_set(-4_in, 70, true);
    stop_intake_auto();
    chassis.pid_wait();
    chassis.pid_turn_set(75, 70);
  } else {
    chassis.pid_drive_set(-2_in, 70, true);
    stop_intake_auto();
    chassis.pid_wait();
    chassis.pid_turn_set(75, 70);
  }
  chassis.pid_wait_until(50);

  stop_intake_auto();

  chassis.pid_wait();
  
  if (chassis.odom_theta_direction_get()) {
    chassis.pid_drive_set(-10_in, 80, true);
    chassis.pid_wait_until(-10);  
  } else {
    chassis.pid_drive_set(-10.5_in, 90, true);
    chassis.pid_wait_until(-2);  
  }
  

  spin_intake_auto(true, 600);


  pros::delay(900);

  stop_intake_auto();

  chassis.pid_drive_set(5_in, 90, true);
  chassis.pid_wait();

  if (chassis.odom_theta_direction_get()) {
      chassis.pid_turn_set(-80, TURN_SPEED);
 
  } else {
      chassis.pid_turn_set(-79, TURN_SPEED);
 
  }

  chassis.pid_wait();

  if (chassis.odom_theta_direction_get()) {
    chassis.pid_drive_set(-38_in, 65, true);
    chassis.pid_wait_until(-32);
  } else {
    chassis.pid_drive_set(-36_in, 90, true);
    chassis.pid_wait_until(-32);
  }

  actuate_back_mogo(true);
  actuate_intake(false);

  chassis.pid_wait_quick();

  chassis.pid_turn_set(155, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(18_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  spin_intake_auto(true, 600);

  chassis.pid_drive_set(7_in, 50, true);
  chassis.pid_wait();

  pros::delay(200);

  if (chassis.odom_theta_direction_get()) {
    chassis.pid_drive_set(-31_in, DRIVE_SPEED, true);
  } else {
    chassis.pid_drive_set(-26_in, DRIVE_SPEED, true);
  }
  chassis.pid_wait();

  chassis.pid_turn_set(220, TURN_SPEED);
  chassis.pid_wait();

  stop_intake_auto();
  spin_intake_auto(true, 600);

  chassis.pid_drive_set(16_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(11_in, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  stop_intake_auto();
  spin_intake_auto(true, 600);

  if (chassis.odom_theta_direction_get()) {
    chassis.pid_turn_set(100, TURN_SPEED);
  } else {
    chassis.pid_turn_set(170, TURN_SPEED);
  }
  chassis.pid_wait();  

  // if (chassis.odom_theta_direction_get()) {
  // chassis.pid_drive_set(14_in, 100, true);
  // } else {
  // chassis.pid_drive_set(16_in, 100, true);

  // }
  // chassis.pid_wait();

  chassis.drive_set(0, 0);

  pros::delay(10000);

  stop_intake_auto();


}

void blue_positive() {
  chassis.odom_theta_flip();
  chassis.odom_theta_direction_get();

  red_positive();
}

void skillz() {

  actuate_intake(false);

  spin_intake_auto(true, 600);

  pros::delay(400);

  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  stop_intake_auto();
  chassis.pid_wait();

  chassis.pid_turn_set(-90, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  actuate_back_mogo(true);

  stop_intake_auto();
  spin_intake_auto(true, 600);

  chassis.pid_drive_set(-8_in, 60, true);
  chassis.pid_wait(); 

  pros::delay(200);

  chassis.pid_turn_set(-10, TURN_SPEED);
  chassis.pid_wait();  

  stop_intake_auto();
  spin_intake_auto(true, 600);

  chassis.pid_drive_set(26_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(200);

  chassis.pid_turn_set(55, TURN_SPEED);
  chassis.pid_wait();

  stop_intake_auto();
  spin_intake_auto(true, 600);

  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(200);

  chassis.pid_drive_set(-2_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(200, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(22_in, 90, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(6_in, 60, true);
  chassis.pid_wait();

  pros::delay(750);

  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(22_in, 85, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(6_in, 60, true);
  chassis.pid_wait();

  pros::delay(400);

  chassis.pid_drive_set(4_in, 60, true);
  chassis.pid_wait();


  pros::delay(1000);

  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(70, TURN_SPEED);
  chassis.pid_wait();

  stop_intake_auto();
  spin_intake_auto(true, 600);

  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-25, TURN_SPEED);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-2);
  actuate_back_mogo(false);
  chassis.pid_wait();  

  stop_intake_auto();

}


///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    // chassis.drive_brake_set(MOTOR_BRAKE_HOLD);`c
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .