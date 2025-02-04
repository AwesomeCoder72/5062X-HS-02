#include "EZ-Template/drive/drive.hpp"
#include "intake.hpp"
#include "ladybrown.hpp"
#include "main.h"
#include "intake.hpp"
#include "pistoncontrol.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
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

  chassis.pid_drive_constants_set(19.0, 0.0, 90.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(2.0, 0.0, 15.0);        // Holds the robot straight while going forward without odom

  // chassis.pid_drive_constants_set(19.0, 0.0, 90.0);         // Fwd/rev constants, used for odom and non odom motions
  // chassis.pid_heading_constants_set(5.0, 0.0, 10.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5); 

  // P, I, D, and Start I
  // chassis.pid_drive_constants_set(10.7, 0.0, 15.7)
  // ;         // Fwd/rev constants, used for odom and non odom motions
  // chassis.pid_heading_constants_set(12.0, 0.0, 17.5);        // Holds the robot straight while going forward without odom
  // chassis.pid_turn_constants_set(3.0, 0.05, 25.0, 15.0);     // Turn in place constants
  // chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  // chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
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
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}



///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  // spin_intake_auto(true, 600);
  // pros::delay(3000);
  // stop_intake_auto();

  // chassis.pid_turn_set(90_deg, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_turn_set(180_deg, TURN_SPEED);
  // chassis.pid_wait();

  // pros::delay(13500);

  
  // chassis.pid_turn_set(90, TURN_SPEED);

  chassis.pid_drive_set(-24, DRIVE_SPEED, true, false);
  // pros::delay(400);
  chassis.pid_wait();
  chassis.drive_set(0, 0);
  // pros::delay(3000);
  


  // chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // chassis.drive_set(0, 0);

  // chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();
}

void elim_auto_2() {
  chassis.odom_theta_set(62.5);

  chassis.pid_drive_set(18_in, 85, true);
  chassis.pid_wait_until(2);

  // actuate_intake(false);

  Intake.move_relative(100000, 600);
  RingLift.move_relative(100000, 400);

  pros::delay(650);

  // chassis.pid_wait_until(11);

  actuate_intake(false);

  stop_intake_auto();

  chassis.pid_wait();

  // chassis.pid_drive_set(-3_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // stop_intake_auto();
  // spin_intake_auto(true, 600, true);

  // pros::delay(1000);

  chassis.pid_turn_set(0, TURN_SPEED);
  
  chassis.pid_wait();

  // drive to alliance stake
  chassis.pid_drive_set(-6, DRIVE_SPEED, true);

  chassis.pid_wait_until(-2);

  actuate_intake(false);

  spin_intake_auto(true, 600);

  chassis.pid_wait();


  pros::delay(750);

  stop_intake_auto();

  chassis.drive_set(0, 0);
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
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  actuate_back_mogo(true);

  spin_intake_auto(true, 600);

  pros::delay(2000);
  stop_intake_auto();

  chassis.pid_turn_set(60, TURN_SPEED);
  chassis.pid_wait();

  LadyBrownMech.set_brake_mode(pros::MotorBrake::hold);

  LadyBrownMech.move_relative(375, 600);
  pros::delay(300);

  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
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