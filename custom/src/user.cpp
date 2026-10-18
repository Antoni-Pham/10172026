#include "vex.h"
#include "motor-control.h"
#include "../include/autonomous.h"
#include "../include/subsystems.h"
#include "../include/robot-config.h"

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  startSubsystems();
  updateDriverSubsystems(false);
  int auton_selected = 1;
  switch(auton_selected) {
    case 1:
      //exampleAuton();
      break;
    case 2:
      //exampleAuton2();
      break;  
    case 3:
      //redGoalRush();
      break;
    case 4:
      break; 
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
  }
}

/* Past attempts */
// // controller_1 input variables (snake_case)
// int ch1, ch2, ch3, ch4;
// bool l1, l2, r1, r2;
// bool button_a, button_b, button_x, button_y;
// bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
// bool toggleauton = false;
// int chassis_flag = 0;
// int clawstate = 0; //0 = Open | 1 = Closed
//
// vex::thread toggleTask([]{
//   toggleOptical.setLightPower(100, pct);
//   while(toggleauton){
//     if ((toggleOptical.value() > 0 && toggleOptical.value() < 30 && toggleOptical.isNearObject())||(controller_1.ButtonX.PRESSED)){
//         vex::wait(300, msec);
//         toggle.stop(hold);
//         waitUntil(!toggleOptical.isNearObject());
//       } else if (!toggleOptical.isNearObject()){
//         toggle.spin(fwd, 12, volt);
//       } wait(10, msec);
// }});
//
// bool isIntaking;
// bool isOuttaking;

void runDriver() {
  // Stop auton subsystems until we incorporate auton functions to aid driver.
  stopSubsystems();

  stopChassis(coast);
  heading_correction = false;

  int ch1 = 0;
  int ch2 = 0;
  int ch3 = 0;
  int ch4 = 0;
  bool r1 = false;
  bool r2 = false;
  bool l1 = false;
  bool l2 = false;
  bool buttonX = false;
  int clawState = 0; //0 = Open | 1 = Closed
  bool toggleOn = false;
  bool brainTimerOn = false;
  bool after1 = false;
  bool after2 = false;
  Brain.resetTimer();

  while (true) {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // default tank drive or replace it with your preferred driver code here: 
    driveChassis(ch3 * 0.12 + ch1 * 0.123, ch3 * 0.12 - ch1 * 0.123);

    // To be re-incorporated to aid driver with auton functions
    // updateDriverSubsystems(l2);

    if (Brain.timer(msec) > 500 && brainTimerOn && after1 == false) {
      lift.spinToPosition(-8, degrees);
      after1 = true;
    }
    if (Brain.timer(msec) > 1500 && brainTimerOn && after2 == false) {
      lift.stop(hold);
      lift.setPosition(0, degrees);
      after2 = true;
    }
    // Open Claw with L1, Close Claw with L2, Lift Down with R1, Lift Up with R2
    if (controller_1.ButtonL1.pressing() && !l1) {
      l1 = true;
      clawRotator.spin(fwd, 4, volt);
      claw.spin(fwd, 5, volt);
      clawState = 1;
    } else if (!controller_1.ButtonL1.pressing() && l1) {
      l1 = false;
    }
    if (controller_1.ButtonL2.pressing() && !l2) {
      l2 = true;
      clawRotator.spin(fwd, -4, volt);
      claw.spin(fwd, 0, volt);
      clawState = 0;
      Brain.resetTimer();

      brainTimerOn = false;
      after1 = false;
      after2 = false;
    } else if (!controller_1.ButtonL2.pressing() && l2) {
      l2 = false;
      clawRotator.spin(fwd, -4, volt);
      claw.spin(fwd, 5, volt);
    }
    if (controller_1.ButtonR1.pressing() && !r1) {
      lift.spin(fwd, -4, volt);
      r1 = true;
    } else if (!controller_1.ButtonR1.pressing() && r1) {
      r1 = false;
    }
    if (controller_1.ButtonR2.pressing() && !r2) {
      r2 = true;
      lift.spin(fwd, 10, volt);
    } else if (!controller_1.ButtonR2.pressing() && r2) {
      r2 = false;
    }
    if (!controller_1.ButtonR1.pressing() && !controller_1.ButtonR2.pressing()) {
      lift.stop(hold);
    }

    // Toggle control: Button X = Toggle on/off
    if(!controller_1.ButtonX.pressing() && buttonX){
      buttonX = false;
    } else if (controller_1.ButtonX.pressing() && !toggleOn && !buttonX) {
      toggle.spin(reverse, 9, volt);
      toggleOn = true;
      buttonX = true;
    } else if (controller_1.ButtonX.pressing() && toggleOn && !buttonX) {
      toggle.stop(hold);
      toggleOn = false;
      buttonX = true;
    }
    
    /* Past attempts */
    // true/false for controller button presses
    // l1 = controller_1.ButtonL1.pressing();
    // l2 = controller_1.ButtonL2.pressing();
    // r1 = controller_1.ButtonR1.pressing();
    // r2 = controller_1.ButtonR2.pressing();
    // button_a = controller_1.ButtonA.pressing();
    // button_b = controller_1.ButtonB.pressing();
    // button_x = controller_1.ButtonX.pressing();
    // button_y = controller_1.ButtonY.pressing();
    // button_up_arrow = controller_1.ButtonUp.pressing();
    // button_down_arrow = controller_1.ButtonDown.pressing();
    // button_left_arrow = controller_1.ButtonLeft.pressing();
    // button_right_arrow = controller_1.ButtonRight.pressing();
    //
    // // Lift control: R1 = Up, R2 = Down
    // if (r1 && !r2) {
    //   lift.spin(fwd, 12, volt);
    // } else if (r2 && !r1) {
    //   lift.spin(fwd, -12, volt);
    // } else {
    //   lift.stop(hold);
    // }
    //
    // Claw control: L1 = Close + Intake, L2 = Open + Outtake
    // if (l1 && !l2) {
    //   clawRotator.spin(fwd, 4, volt);
    //   claw.spin(fwd, 8, volt);
    //   clawstate = 1;
    // } else if (l2 && !l1) {
    //   clawRotator.spin(fwd, -4, volt);
    //   claw.spin(reverse, 8, volt);
    //   clawstate = 0;
    // } else {
    //   // Neither button pressed: keep claw open and intake
    //   clawRotator.stop(hold);
    //   claw.spin(fwd, 4, volt);
    //   clawstate = 0;
    // }
    //
    // if(r2){
    //   lift.spin(fwd, 12, volt);
    // } else if(r1){
    //   lift.spin(fwd, -12, volt);
    // } else if(l1){
    //   claw.spin(fwd, -6, volt);
    //   clawRotator.spin(fwd, -6, volt);
    // } else if(l2){
    //   clawRotator.spin(fwd, 6, volt);
    //   clawstate = 1;
    //   claw.spin(fwd, 12, volt);
    // } else{
    //   lift.stop(hold);
    //   clawRotator.stop(hold);
    // }
  }
}

void runPreAutonomous() {
    // Initializing   mvc  Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // To be re-enabled when starting auton work.
  // startSubsystems();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);
  
  // odom tracking
  resetChassis();
  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }
}