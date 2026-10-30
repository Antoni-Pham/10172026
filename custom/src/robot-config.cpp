#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller controller_1 = controller(primary);

// IMPORTANT: Remember to modify the example motors according to the guide. 
// Also remember to add respective device declarations to custom/include/robot-config.h
// Format: motor(port, gearSetting, reversed)
// gearSetting is one of the following: ratio36_1(red), ratio18_1(green), ratio6_1(blue)
// all chassis motors should be reversed appropriately so that they spin vertical when given a positive voltage input
// such as driveChassis(12, 12)


motor left_chassis1 = motor(PORT11, ratio6_1, true);
motor left_chassis2 = motor(PORT12, ratio6_1, true);
motor left_chassis3 = motor(PORT14, ratio18_1, true);
motor_group left_chassis = motor_group(left_chassis2, left_chassis1, left_chassis3);

motor right_chassis1 = motor(PORT8, ratio6_1, false);
motor right_chassis2 = motor(PORT9, ratio6_1, false);
motor right_chassis3 = motor(PORT17, ratio18_1, false);
motor_group right_chassis = motor_group(right_chassis2, right_chassis1, right_chassis3);

motor lift = motor(PORT6, ratio36_1, true);
// motor liftL = motor(PORT1, ratio36_1, false);
// motor_group lift = motor_group(liftL, liftR);

motor toggle = motor(PORT2, ratio18_1, true);

motor clawL = motor(PORT10, ratio18_1, false);
motor clawR = motor(PORT3, ratio18_1, true);
motor clawRotator = motor(PORT4, ratio18_1, true);
motor_group claw = motor_group(clawL, clawR);

inertial inertial_sensor = inertial(PORT7);
optical toggleOptical = optical(PORT15);
distance clawDetect = distance(PORT5);



// Format is rotation(port, reversed)
// just set these to random ports if you don't use tracking wheels
rotation horizontal_tracker = rotation(PORT22, true);
rotation vertical_tracker = rotation(PORT18, true);

// Distance reset sensors 
// Set these to random ports if you are not using distance resets
distance front_sensor = distance(PORT16);
distance left_sensor = distance(PORT20);
distance right_sensor = distance(PORT13);
distance back_sensor = distance(PORT19);


// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================

// Distance between the middles of the left and right wheels of the drive (in inches)
double distance_between_wheels = 11.25;

// motor to wheel gear ratio * wheel diameter (in inches) * pi
double wheel_distance_in = (36.0 / 48.0) * 2.75 * M_PI;

// PID Constants for movement
// distance_* : Linear PID for straight driving
// sensor_distance_* : PID for driving to a distance-sensor gap
// turn_*     : PID for turning in place
// turn_to_point_* : PID for turning to face a field point
// tip_*      : PID for chassis pitch correction
// heading_correction_* : PID for heading correction during linear movement
double distance_kp = 1.3, distance_ki = 0.0, distance_kd = 8;
double sensor_distance_kp = 2, sensor_distance_ki = 0.0, sensor_distance_kd = 7.5;
double turn_kp = 0.5, turn_ki = 0, turn_kd = 2.7;
double turn_to_point_kp = 0.5, turn_to_point_ki = 0, turn_to_point_kd = 3.4;
double tip_kp = 0.8, tip_ki = 0.0, tip_kd = 1.5;
double heading_correction_kp = 0.8, heading_correction_ki = 0, heading_correction_kd = 4;

// Enable or disable the use of tracking wheels
bool using_horizontal_tracker = false;  // Set to true if a horizontal tracking wheel is installed and used for odometry
bool using_vertical_tracker = true;   // Set to true if a vertical tracking wheel is installed and used for odometry

// IGNORE THESE IF YOU ARE NOT USING TRACKING WHEELS
// These comments are in the perspective of a top down view of the robot when the robot is facing vertical
// Vertical distance from the center of the bot to the horizontal tracking wheel (in inches, positive is when the wheel is behind the center)
double horizontal_tracker_dist_from_center = 2.71875;
// Horizontal distance from the center of the bot to the vertical tracking wheel (in inches, positive is when the wheel is to the right of the center)
double vertical_tracker_dist_from_center = 0.5;
double horizontal_tracker_diameter = 1.975; // Diameter of the horizontal tracker wheel (in inches)
double vertical_tracker_diameter = 1.975; // Diameter of the vertical tracker wheel (in inches)

// Distance Reset setup
// If you are not using all four distance sensors, just set the unused ones to 0
// If you are not using distance resets these values will be ignored
// Add your sensor offsets here
// If a sensor is dead-center but 6.5" forward relative to the robot's center:
// double front_sensor_offsetX = 0.0; 
// double front_sensor_offsetY = 6.5;
// For the X offset, a negative value indicates that the sensor is located on the left side of the robot, while a positive value indicates that it is on the right side. 

double front_sensor_offsetX = -6.5; 
double front_sensor_offsetY = 2;

double left_sensor_offsetX = -6.0; 
double left_sensor_offsetY = 0.7;

double right_sensor_offsetX = 5.75;
double right_sensor_offsetY = 1.0;

double back_sensor_offsetX = -6.0;
double back_sensor_offsetY = -7.0;

// ============================================================================
// ADVANCED TUNING (OPTIONAL)
// ============================================================================

bool heading_correction = true; // Use heading correction when the bot is stationary

// Set to true for more accuracy and smoothness, false for more speed
bool dir_change_start = false;   // Less accel/decel due to expecting direction change at start of movement
bool dir_change_end = false;     // Less accel/decel due to expecting direction change at end of movement

double min_output = 10; // Minimum output voltage to motors while chaining movements

// Maximum allowed change in voltage output per 10 msec during movement
double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 10;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 10;

// Prevents too much slipping during boomerang movements
// Decrease if there is too much drifting and inconsistency during boomerang
// Increase for more speed during boomerang
double chase_power = 5;

// ============================================================================
// DO NOT CHANGE ANYTHING BELOW
// ============================================================================

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}
