#include "main.h"
#include <concepts>
#include <functional>
#include <memory>
#include <string>
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "drivecontrol.hpp"
#include "intake.hpp"
#include "ladybrown.hpp"
#include "pistoncontrol.hpp"
#include "drivecontrol.hpp"
#include "pros/colors.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////


// RADIO PORT - 13

/*
	MOTOR PORT DEFINITIONS
*/

#define RING_LIFT_MOTOR_PORT 10
#define INTAKE_MOTOR_PORT 15

#define LADY_BROWN_MOTOR_PORT 1

#define LADY_BROWN_ROTATION_SENSOR_PORT 3

#define DRIVE_LB_PORT 20 // 17
#define DRIVE_LT_PORT 7
#define DRIVE_LF_PORT 2

#define DRIVE_RB_PORT 18 
#define DRIVE_RT_PORT 9
#define DRIVE_RF_PORT 12

#define BACK_MOGO_ACTUATOR_PORT 'd'
#define INTAKE_ACTUATOR_PORT 'a'
#define RIGHT_DOINKER_ACTUATOR_PORT 'b'
#define LEFT_DOINKER_ACTUATOR_PORT 'c'



// #define RING_LIFT_MOTOR_PORT 20 
// #define INTAKE_MOTOR_PORT 4

// #define DRIVE_LB_PORT 17
// #define DRIVE_LT_PORT 19
// #define DRIVE_LF_PORT 18

// #define DRIVE_RB_PORT 15
// #define DRIVE_RT_PORT 11
// #define DRIVE_RF_PORT 12

/*
	DRIVE MOTOR INITIALIZATIONS
*/

pros::Rotation LadyBrownRotationSensor(LADY_BROWN_ROTATION_SENSOR_PORT);

pros::Motor LadyBrownMech(-LADY_BROWN_MOTOR_PORT);

pros::Motor drive_lb(DRIVE_LB_PORT);
pros::Motor drive_lt(DRIVE_LT_PORT);
pros::Motor drive_lf(DRIVE_LF_PORT);

pros::Motor drive_rb(DRIVE_RB_PORT);
pros::Motor drive_rt(DRIVE_RT_PORT);
pros::Motor drive_rf(DRIVE_RF_PORT);

/*
	DRIVE MOTOR GROUP INITIALIZATIONS
*/

pros::MotorGroup drive_left({DRIVE_LT_PORT, -DRIVE_LB_PORT,  -DRIVE_LF_PORT});
pros::MotorGroup drive_right({DRIVE_RB_PORT, -DRIVE_RT_PORT, DRIVE_RF_PORT});


/*
	IMU PORT DEFINITIONS
*/

#define IMU_PORT 8

// ez::Inertial imu(IMU_PORT);

/*
	SMART SENSOR PORT DEFINTIONS
*/

#define RING_OPTICAL_SENSOR_PORT 6

/*
	DIGITAL PORT DEFINITIONS
*/

// #define CATA_LIMIT_SWITCH_PORT 'H'
// #define AUTON_POT_PORT 'E'

// #define INTAKE_ACTUATOR_PORT 'G'
// #define RIGHT_FRONT_WING_ACTUATOR_PORT 'A'
// #define LEFT_FRONT_WING_ACTUATOR_PORT 'B'



/*
	CONTROLLER BUTTON DEFINITIONS
*/

// #define AUTON_SELECT_BUTTON pros::E_CONTROLLER_DIGITAL_UP

#define INTAKE_INTAKE_BUTTON pros::E_CONTROLLER_DIGITAL_L1
#define INTAKE_OUTTAKE_BUTTON pros::E_CONTROLLER_DIGITAL_L2

#define SHIFT_BUTTON pros::E_CONTROLLER_DIGITAL_Y
#define LADY_BROWN_DOWN_BUTTON pros::E_CONTROLLER_DIGITAL_DOWN
#define LADY_BROWN_NEXT_BUTTON pros::E_CONTROLLER_DIGITAL_R1

#define ACTUATE_MOGO_BUTTON pros::E_CONTROLLER_DIGITAL_R2
#define ACTUATE_INTAKE_BUTTON pros::E_CONTROLLER_DIGITAL_RIGHT
#define ACTUATE_RIGHT_DOINKER_BUTTON pros::E_CONTROLLER_DIGITAL_B
#define ACTUATE_LEFT_DOINKER_BUTTON pros::E_CONTROLLER_DIGITAL_DOWN



// #define UP_MATCH_LOAD_SPEED_BUTTON pros::E_CONTROLLER_DIGITAL_LEFT
// #define DOWN_MATCH_LOAD_SPEED_BUTTON pros::E_CONTROLLER_DIGITAL_DOWN

#define LIMIT_DRIVE_SPEED_BUTTON pros::E_CONTROLLER_DIGITAL_LEFT

/*
	CONTROLLER ARCADE DRIVE AXES
*/

#define ARCADE_DRIVE_FORWARD_BACKWARD_CHANNEL 

/*
	CONTROLLER DEFINITION
*/

pros::Controller controller (pros::E_CONTROLLER_MASTER);

/*
	MOTOR INITIALIZATIONS
*/

pros::Motor Intake(-INTAKE_MOTOR_PORT);
pros::Motor RingLift(RING_LIFT_MOTOR_PORT);

/*
	SENSOR INITIALIZATIONS
*/

pros::Optical RingOptical(RING_OPTICAL_SENSOR_PORT);

/*
	PISTON INITIALIZATIONS
*/

pros::adi::Pneumatics BackMogoActuator(BACK_MOGO_ACTUATOR_PORT, false);
pros::adi::Pneumatics IntakeActuator(INTAKE_ACTUATOR_PORT, true);
pros::adi::Pneumatics RightDoinkerActuator(RIGHT_DOINKER_ACTUATOR_PORT, false);
pros::adi::Pneumatics LeftDoinkerActuator(LEFT_DOINKER_ACTUATOR_PORT, false);

// ez::Piston BackMogoActuator('a', false);

// Chassis constructor
// ez::Drive chassis(
//     // These are your drive motors, the first motor is used for sensing!
//     {-DRIVE_LF_PORT, DRIVE_LT_PORT, -DRIVE_LB_PORT},     // Left Chassis Ports (negative port will reverse it!)
//     {DRIVE_RF_PORT, -DRIVE_RT_PORT, DRIVE_RB_PORT},  // Right Chassis Ports (negative port will reverse it!)

//     IMU_PORT,      // IMU Port
//     3.3,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
//     450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-DRIVE_LB_PORT, -DRIVE_LF_PORT, DRIVE_LT_PORT },     // Left Chassis Ports (negative port will reverse it!)
    {-DRIVE_RT_PORT, DRIVE_RB_PORT, DRIVE_RF_PORT},  // Right Chassis Ports (negative port will reverse it!)

    IMU_PORT,      // IMU Port
     3.30,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


// 0 = off, 
// 1 = intake & ring lift in, 
// 2 = intake & ring lift out, 
// 3 = intake only in, 
// 4 = ring lift only in,
// 5 = intake ring until optical sensor senses
// 6 = ring lift only out

bool ActivelyColorSorting = false;
bool ringInPosition = false;

float currentProximity = RingOptical.get_proximity();
float lastProximity = currentProximity;

bool valueWithinRange(int value, int target, int range) {
	return ((value > target - range) && (value < target + range));
  }

int checkRingColor() {
	// 0 = no ring, 1 = red ring, 2 = blue ring
	if (valueWithinRange(RingOptical.get_hue(), 10, 8)) {
		return 1;
	} else if (valueWithinRange(RingOptical.get_hue(), 210, 8)) {
		return 2;
	} else {
		return 0;
	}
}



void IntakeControlTaskFunction () {
  intakeThrottle = defaultIntakeThrottle;
	while (true) {
		// pros::lcd::print(0, "Hue: %f", RingOptical.get_hue());
    // pros::lcd::print(0, "Proximity: %d`", RingOptical.get_proximity());
    int currentRingColor = checkRingColor();
    currentProximity = RingOptical.get_proximity();
		// pros::lcd::print(1, "Check Ring Color: %d", currentRingColor);
    // pros::lcd::print(1, "Intake Input State: %d", IntakeInputState);
		if (ColorSorterToggle == 0) {
			pros::lcd::print(4, "Color sorter off");
		} else if (ColorSorterToggle == 1) {
			pros::lcd::print(4, "Scheduled to sort out Red");
		} else if (ColorSorterToggle == 2) {
			pros::lcd::print(4, "Scheduled to sort out Blue");
		}
		
			// RingLift.move_voltage(12000);
		if (IntakeInputState == 0) {
			RingLift.move_voltage(0);
      Intake.move_voltage(0);
		} else if (IntakeInputState == 1) {
			if (((currentRingColor == 1 && ColorSorterToggle == 1) || 
				(currentRingColor == 2 && ColorSorterToggle == 2)) ||
				(ActivelyColorSorting))  {
				pros::lcd::print(2, "sorting out ring");
				ActivelyColorSorting = true;
        bool ringHasPassed = false;

        // RingLift.move_relative(10000, 600);

        while (!ringHasPassed) {
          RingLift.move_voltage(12000);
          if ((currentRingColor != checkRingColor())) {
            RingLift.move_relative(-10000, 600);
            pros::delay(80);
            ringHasPassed = true;
          } 
        }
				
				ActivelyColorSorting = false;
				pros::lcd::print(2, "");
			} else {
        Intake.move_voltage(12000*intakeThrottle);
				RingLift.move_voltage(12000*intakeThrottle);
			}
		} else if (IntakeInputState == 2) {
      Intake.move_voltage(-12000*intakeThrottle);
			RingLift.move_voltage(-12000*intakeThrottle);
    } else if (IntakeInputState == 3) {
      Intake.move_voltage(12000*intakeThrottle);
      RingLift.move_voltage(0);
    } else if (IntakeInputState == 4) {
      Intake.move_voltage(0);
      RingLift.move_voltage(12000*intakeThrottle);
		} else if (IntakeInputState == 5) {
      ringInPosition = false;
      // if (currentRingColor == 0) {
        

        while ((!ringInPosition) && IntakeInputState == 5) {
          // RingLift.move_voltage(6000);
          if (checkRingColor() == 1 || checkRingColor() == 2) {
              // RingLift.move_relative(3000, 600);
              // pros::delay(200);
              ringInPosition = true;
          } else {
            Intake.move_voltage(12000*intakeThrottle);
            RingLift.move_voltage(12000*intakeThrottle);
          }
        }
        IntakeInputState = 0;
        // Intake.move_voltage(0);
        // RingLift.move_voltage(0);
    } else if (IntakeInputState == 6) {
      Intake.move_voltage(0);
      RingLift.move_voltage(-12000*intakeThrottle);
    }
    lastProximity = currentProximity;
		pros::delay(20);
		// pros::lcd::clear();
	}
}

void IntakeControl() {
	float intake_sees_ring_position = 0;
	int intake_sees_ring = 0;
	int intake_sees_ring_last = 0;
	float intake_begins_moving_backwards_position = 0;
  string screen_text = "";
  bool color_sorting = false;
	while (true) {
		
			// intake_begins_moving_backwards_position = RingLift.get_position();

			// if (!(intake_sees_ring_position - 500 > RingLift.get_position())) {
			// 	RingLift.move_velocity(-400);
			// }}
		if (valueWithinRange(RingOptical.get_hue(), 10, 15) && RingOptical.get_proximity() > 220) {
			intake_sees_ring = 1;
			// controller.set_text(0, 0, "Ring Detected");

    } else if (valueWithinRange(RingOptical.get_hue(), 200, 25) && RingOptical.get_proximity() > 220) {
			intake_sees_ring = 2; 
		} else {
			intake_sees_ring = 0;
			// controller.set_text(0, 0, "No Ring Detected");

			spin_intake_driver(controller.get_digital(INTAKE_INTAKE_BUTTON),
					   controller.get_digital(INTAKE_OUTTAKE_BUTTON));
		}

				// ...existing code...
    screen_text = std::to_string(intake_sees_ring);
if (!pros::competition::is_connected()) {
    // ez::screen_print(screen_text, 4);

    screen_text = std::to_string(intake_sees_ring_last);

    // ez::screen_print(screen_text, 5);

    if (intake_sees_ring != 0 && intake_sees_ring_last == 0) {
    

      ez::screen_print("color sorting", 6);
    }
				// RingLift.move_voltage(12000);
      stop_intake_auto();
      // color_sort();

          pros::delay(500);
      
        
			} else { if (!pros::competition::is_connected()) {ez::screen_print("", 6);} }
    
		// pros::lcd::print(2, 
								//    "R: %d\nG: %d\nB: %d", 
								//    RingOptical.get_raw().red, 
								//    RingOptical.get_raw().green, 
								//    RingOptical.get_raw().blue);
		// hue value
		// pros::lcd::print(1, "H: %f", pros::c::optical_get_hue(1));
		// ...existing code...controller.set_text(0, 0, std::to_string(RingOptical.get_rgb().blue));
		intake_sees_ring_last = intake_sees_ring;
		// controller.clear();
		pros::delay(5);
	}
}

void runAutonWithTasks() {
  pros::Task liftControlTask([]{
    while (true) {
        liftControl();
        // ez::screen_print(std::to_string(chassis.g), 1);
        pros::delay(10);
        ez::screen_print("", 1);

    }

    });
    pros::Task intakeControlTask([]{
      while (true) {
          IntakeControlTaskFunction();
          // ez::screen_print(std::to_string(LadyBrownRotationSensor.get_position()), 1);
          pros::delay(10);
          // ez::screen_print("", 1);
      }
      
      }
    );}


void initialize() {
  // Print our branding over your terminal :D
  // ez::ez_template_print();

  // pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  

  


  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
    {"red goal safe alliance and stack", [] {runAutonWithTasks(); red_goal_safe_alliance_and_stack();}},
    {"blue ring safe alliance and stack", [] {runAutonWithTasks(); blue_ring_safe_alliance_and_stack();}},
    {"red ring safe alliance and stack", [] {runAutonWithTasks(); red_ring_safe_alliance_and_stack();}},
    {"blue goal safe alliance and stack", [] {runAutonWithTasks(); blue_goal_safe_alliance_and_stack();}},
    {"blue goal safe no stack", [] {runAutonWithTasks(); blue_goal_safe_no_stack();}},
    {"red ring safe no stack", [] {runAutonWithTasks(); red_ring_safe_no_stack();}},
    {"red goal safe no stack", [] {runAutonWithTasks(); red_goal_safe_no_stack();}},
    {"blue ring safe no stack", [] {runAutonWithTasks(); blue_ring_safe_no_stack();}},
    {"red ring six ring new", [] {runAutonWithTasks(); red_ring_six_ring_new();}},
    {"blue ring six ring new", [] {runAutonWithTasks(); blue_ring_six_ring_new();}},
    {"blue ring six ring", [] {runAutonWithTasks(); blue_ring_six_ring();}},
    {"no final ring state elim goal red with center ring", [] {runAutonWithTasks(); state_elim_goal_red_with_center_ring_no_final_ring();}},
    {"state elim goal red with center ring", [] {runAutonWithTasks(); state_elim_goal_red_with_center_ring();}},
    {"state elim goal red", [] {runAutonWithTasks(); state_elim_goal_red();}},
    {"state skillz", []{pros::Task liftControlTask([]{
      while (true) {
          liftControl();
          // ez::screen_print(std::to_string(LadyBrownRotationSensor.get_position()), 1);
          pros::delay(10);
          // ez::screen_print("", 1);
      }

      });
      pros::Task intakeControlTask([]{
        while (true) {
            IntakeControlTaskFunction();
            // ez::screen_print(std::to_string(LadyBrownRotationSensor.get_position()), 1);
            pros::delay(10);
            // ez::screen_print("", 1);
        }
        
        }
      );
      state_skillz();
      }},{"state_center_grab_goal_blue", [] {runAutonWithTasks(); state_center_grab_goal_blue();}},
    {"state_center_grab_goal_red", [] {runAutonWithTasks(); state_center_grab_goal_red();}},
    {"state red goal no alliance", [] {runAutonWithTasks(); state_elim_goal_red_no_alliance_stake();}},
    
      {"state elim goal red", [] {runAutonWithTasks(); state_elim_goal_red();}},
    {"state full awp red", [] {runAutonWithTasks(); state_full_awp_red();}},
    {"state full awp blue", [] {runAutonWithTasks(); state_full_awp_blue();}},
    
    
    
    
    {"goal side center ring grab auto", []{pros::Task liftControlTask([]{
      while (true) {
          liftControl();
          // ez::screen_print(std::to_string(LadyBrownRotationSensor.get_position()), 1);
          pros::delay(10);
          // ez::screen_print("", 1);
      }

      });
      pros::Task intakeControlTask([]{
        while (true) {
            IntakeControlTaskFunction();
            // ez::screen_print(std::to_string(LadyBrownRotationSensor.get_position()), 1);
            pros::delay(10);
            // ez::screen_print("", 1);
        }
        
        }
      );
      od_new_red_right_center_ring_grab();
      }},
    {"odom right red center grab", od_red_right_center_ring_grab_with_alliance_stake},
    {"odom test", drive_example},
    
    {"awp blue left center ring grab", blue_left_center_ring_grab_awp},
    {"awp red right center ring grab", red_right_center_ring_grab_awp}, 
    
    {"blue left solo awp", blue_left_solo_awp},
     

    
    
    {"red right solo awp", red_right_solo_awp},
    {"blue left center ring grab", blue_left_center_ring_grab}, 

    

    {"red right center ring grab", red_right_center_ring_grab}, 
    {"Safe Left", safe_left},
    {"left elim", left_elim},
    {"skillz2" , []{pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            // ez::screen_print(std::to_string(LadyBrownRotationSensor.get_position()), 1);
            pros::delay(10);
            ez::screen_print("", 1);
        }
        }
        );
        skillz2();
        }},
    {"GYATTTTT", elim_auto_2},
    {"red right", red_right},
    {"sigma auto 2", elim_auto_3},
    {"drive and score", drive_and_score},
   
  
    
    {"XENOFOX", drive_example}, 
        
    
    {"full awp sig", full_awp_sig},
    
      {"skills sig", skills_sig},

      {"Blue Positive", blue_positive},
      {"Red Positive", red_positive},
      {"SKILLZ", skillz},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  pros::delay(100);
  chassis.initialize();
  ez::as::initialize();
  // pros::Task IntakeControlTask(IntakeControl);
  RingOptical.set_led_pwm(50);

  

  controller.rumble(chassis.drive_imu_calibrated() ? "." : "...");  // Rumble the controller if the IMU is calibrated

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}


/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  
      // Calls selected auton from autonomous selector

  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();

  // IntakeControlTask.delay(15500);
  // blue_positive();
  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (controller.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (controller.get_digital(DIGITAL_X) && controller.get_digital(DIGITAL_UP)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

bool actuate_mogo_btn_pressed = false;
bool actuate_mogo_btn_pressed_last = false;

bool actuate_intake_btn_pressed = false;
bool actuate_intake_btn_pressed_last = false;

bool lady_brown_next_btn_pressed = false;
bool lady_brown_next_btn_pressed_last = false;

bool lady_brown_shift_btn_pressed = false;
bool lady_brown_shift_btn_pressed_last = false;


void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  LadyBrownMech.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.drive_set(0, 0);

  pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            // ez::screen_print(std::to_string(LadyBrownRotationSensor.get_position()), 1);
            pros::delay(10);
            ez::screen_print("", 1);
        }
    });

    pros::Task IntakeControlTask(IntakeControlTaskFunction);
  

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    // chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    split_arcade(controller.get_analog(ANALOG_LEFT_X),
                     controller.get_analog(ANALOG_LEFT_Y), 
                     controller.get_analog(ANALOG_RIGHT_X), 
                     controller.get_analog(ANALOG_RIGHT_Y), 
                     15, 
                     10, 
                     80, 
                     controller.get_digital(LIMIT_DRIVE_SPEED_BUTTON));

    // Intake control
    if (controller.get_digital(INTAKE_INTAKE_BUTTON)) {
      if (controller.get_digital(SHIFT_BUTTON)) {
        IntakeInputState = 5;
      } else {
        IntakeInputState = 1;
      }
		} else if (controller.get_digital(INTAKE_OUTTAKE_BUTTON)) {
			IntakeInputState = 2;
    // } else if  && controller.get_digital(INTAKE_INTAKE_BUTTON)) {
    //   IntakeInputState = 5;
		} else {
			IntakeInputState = 0;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			ColorSorterToggle += 1;
			if (ColorSorterToggle > 2) {
				ColorSorterToggle = 0;
			}
		}

    

    // spin_intake_driver(controller.get_digital(INTAKE_INTAKE_BUTTON), controller.get_digital(INTAKE_OUTTAKE_BUTTON));

    // spin_lady_brown_driver(controller.get_digital(LADY_BROWN_UP_BUTTON), controller.get_digital(LADY_BROWN_DOWN_BUTTON));

    // Mogo actuator control

    if (controller.get_digital(ACTUATE_MOGO_BUTTON)) {
				actuate_mogo_btn_pressed = true;
			} else {
				actuate_mogo_btn_pressed = false;
		}

		if (actuate_mogo_btn_pressed && ! actuate_mogo_btn_pressed_last) {
		  if (!back_mogo_actuated_value) actuate_back_mogo(true);
		  else actuate_back_mogo(false);
		}

    actuate_mogo_btn_pressed_last = actuate_mogo_btn_pressed;
 
    if (controller.get_digital(ACTUATE_INTAKE_BUTTON)) {
				actuate_intake_btn_pressed = true;
			} else {
				actuate_intake_btn_pressed = false;
		}

		if (actuate_intake_btn_pressed && ! actuate_intake_btn_pressed_last) {
		  if (!intake_actuated_value) actuate_intake(true);
		  else actuate_intake(false);
		}

    if (controller.get_digital_new_press(ACTUATE_LEFT_DOINKER_BUTTON)) {
      if (!left_doinker_actuated_value) actuate_left_doinker(true);
      else actuate_left_doinker(false);
    }

    if (controller.get_digital_new_press(ACTUATE_RIGHT_DOINKER_BUTTON)) {
      if (!right_doinker_actuated_value) actuate_right_doinker(true);
      else actuate_right_doinker(false);
    }

    actuate_intake_btn_pressed_last = actuate_intake_btn_pressed;

    // if (controller.get_digital(LADY_BROWN_NEXT_BUTTON)) {
    //   lady_brown_next_btn_pressed = true;
    // } else {
    //   lady_brown_next_btn_pressed_last = false;
    // }

    // if (controller.get_digital(LADY_BROWN_SHIFT_BUTTON)) {
    //   lady_brown_shift_btn_pressed = true;
    // } else {
    //   lady_brown_shift_btn_pressed_last = false;
    // }

    // if (lady_brown_shift_btn_pressed) {
    //   if (lady_brown_next_btn_pressed) {
    //     nextState(3);
    //   } else {
    //     nextState(0);
    //   }
    // } else if (lady_brown_next_btn_pressed) {
    //   nextState();
    // }

    if (controller.get_digital_new_press(LADY_BROWN_NEXT_BUTTON)) {
      if (currState == 1) {
        if (controller.get_digital(INTAKE_INTAKE_BUTTON)) {
          nextState(10);



        }
      }
      if (controller.get_digital(SHIFT_BUTTON)) {
        if (currState == 0) {
          toggleThrottleTargetSpeed = false;
          nextState(2);
          if (IntakeActuator.is_extended()) {
            actuate_intake(true);
          }
        } else if (currState == 1) {
          toggleThrottleTargetSpeed = true;
          nextState(2);
        } else if (currState == 2) {
          toggleThrottleTargetSpeed = false;
          nextState(3);
          }
        } else {
         toggleThrottleTargetSpeed = false;
        nextState();
      } }


    // if ((controller.get_digital(SHIFT_BUTTON)) && (controller.get_digital_new_press(LADY_BROWN_NEXT_BUTTON))) {
      // nextState();
    // } else if (controller.get_digital_new_press(LADY_BROWN_SHIFT_BUTTON)) {
      
    // } else 


    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
