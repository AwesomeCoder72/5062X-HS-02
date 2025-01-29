#include "main.h"
#include <string>
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "drivecontrol.hpp"
#include "intake.hpp"
#include "ladybrown.hpp"
#include "pistoncontrol.hpp"
#include "drivecontrol.hpp"
#include "pros/colors.hpp"
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

#define RING_LIFT_MOTOR_PORT 15
#define INTAKE_MOTOR_PORT 20

#define LADY_BROWN_MOTOR_PORT 13

#define LADY_BROWN_ROTATION_SENSOR_PORT 11

#define DRIVE_LB_PORT 17
#define DRIVE_LT_PORT 7
#define DRIVE_LF_PORT 6

#define DRIVE_RB_PORT 9 // out sometimes
#define DRIVE_RT_PORT 5 // questionable
#define DRIVE_RF_PORT 14



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

pros::MotorGroup drive_left({-DRIVE_LB_PORT, DRIVE_LT_PORT, -DRIVE_LF_PORT});
pros::MotorGroup drive_right({DRIVE_RB_PORT, -DRIVE_RT_PORT, DRIVE_RF_PORT});


/*
	IMU PORT DEFINITIONS
*/

#define IMU_PORT 16

/*
	SMART SENSOR PORT DEFINTIONS
*/

#define RING_OPTICAL_SENSOR_PORT 1

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

#define LADY_BROWN_UP_BUTTON pros::E_CONTROLLER_DIGITAL_RIGHT
#define LADY_BROWN_DOWN_BUTTON pros::E_CONTROLLER_DIGITAL_DOWN
#define LADY_BROWN_NEXT_BUTTON pros::E_CONTROLLER_DIGITAL_R1

#define ACTUATE_MOGO_BUTTON pros::E_CONTROLLER_DIGITAL_R2
#define ACTUATE_INTAKE_BUTTON pros::E_CONTROLLER_DIGITAL_RIGHT



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
pros::Motor RingLift(-RING_LIFT_MOTOR_PORT);

/*
	SENSOR INITIALIZATIONS
*/

pros::Optical RingOptical(RING_OPTICAL_SENSOR_PORT);

/*
	PISTON INITIALIZATIONS
*/

pros::adi::Pneumatics BackMogoActuator('b', false);
pros::adi::Pneumatics IntakeActuator('c', true);

// ez::Piston BackMogoActuator('a', false);

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-DRIVE_LF_PORT, DRIVE_LT_PORT, -DRIVE_LB_PORT},     // Left Chassis Ports (negative port will reverse it!)
    {DRIVE_RF_PORT, -DRIVE_RT_PORT, DRIVE_RB_PORT},  // Right Chassis Ports (negative port will reverse it!)

    IMU_PORT,      // IMU Port
    3.3,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

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

bool valueWithinRange(int value, int target, int range) {
  return ((value > target - range) && (value < target + range));
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
		if (valueWithinRange(RingOptical.get_hue(), 10, 10) && RingOptical.get_proximity() > 220) {
			intake_sees_ring = 1;
			// controller.set_text(0, 0, "Ring Detected");

    } else if (valueWithinRange(RingOptical.get_hue(), 200, 20) && RingOptical.get_proximity() > 220) {
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

const int numStates = 4;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {14250, 16220, 23000, 27750};
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
    double kP = 0.025;
    double kD = 0.0; 
    double error = target - LadyBrownRotationSensor.get_position();
    double derivative = (error-last_error);
    double velocity = kP * error + kD * derivative;
    last_error = error;
    // if (LadyBrownRotationSensor.get_position() > states[numStates] + 50){

    // }
    LadyBrownMech.move(velocity);
}

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
      {"Drive\n\nDrive forward and come back", drive_example},
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
  RingOptical.set_led_pwm(20);

  

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
    if (controller.get_digital(DIGITAL_B) && controller.get_digital(DIGITAL_DOWN)) {
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

void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  LadyBrownMech.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            ez::screen_print(std::to_string(LadyBrownRotationSensor.get_position()), 1);
            pros::delay(10);
            ez::screen_print("", 1);
        }
    });


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

    spin_intake_driver(controller.get_digital(INTAKE_INTAKE_BUTTON), controller.get_digital(INTAKE_OUTTAKE_BUTTON));

    spin_lady_brown_driver(controller.get_digital(LADY_BROWN_UP_BUTTON), controller.get_digital(LADY_BROWN_DOWN_BUTTON));

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

    actuate_intake_btn_pressed_last = actuate_intake_btn_pressed;

    if (controller.get_digital_new_press(LADY_BROWN_NEXT_BUTTON)) {
      nextState();
    }


    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
