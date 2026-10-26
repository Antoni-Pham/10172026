#include "vex.h"
#include "motor-control.h"
#include "../include/user.h"
#include "../include/autonomous.h"
#include "../include/subsystems.h"
#include "../include/robot-config.h"

int auton_selected = 1;

// Modify autonomous, driver, or pre-auton code below
competition Competition;
void runAutonomous() {
  
  //startSubsystems();
  //updateDriverSubsystems(false);
  
  lift.setPosition(5, degrees);
  switch(auton_selected){
    case 1:
      Pin3Quad1(1);
      break;
    case 2:
      exampleAuton2();
      break;  
    case 3:
      Skills();
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

volatile int selectedRollerColor = static_cast<int>(RollerColor::Red);

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
  lift.setVelocity(100, percent);
  while (true) {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // default tank drive or replace it with your preferred driver code here: 
    driveChassis(ch3 * 0.08 + ch1 * 0.08, ch3 * 0.08 - ch1 * 0.08);

    // To be re-incorporated to aid driver with auton functions
    // updateDriverSubsystems(l2);

    if (Brain.timer(msec) > 500 && brainTimerOn && after1 == false) {
      lift.setVelocity(85, percent);
      lift.spinToPosition(-8, degrees, false);
      after1 = true;
    }
    if (Brain.timer(msec) > 1800 && brainTimerOn && after2 == false) {
      brainTimerOn = false;
      after1 = false;
      lift.stop(hold);
      lift.setPosition(0, degrees);
      after2 = true;
    }
    // Open Claw with L1, Close Claw with L1, Lift Down with R1, Lift Up with R2
    if (controller_1.ButtonL1.pressing() && !l1 && clawState == 0) {
      l1 = true;
      clawRotator.spin(fwd, 4, volt);
      claw.spin(fwd, 5, volt);
      clawState = 1;
    } else if (controller_1.ButtonL1.pressing() && !l1 && clawState == 1) {
      l1 = true;
      clawRotator.spin(fwd, -4, volt);
      claw.stop(coast);
      clawState = 0;
      Brain.resetTimer();

      brainTimerOn = true;
      after1 = false;
      after2 = false;
    } else if (!controller_1.ButtonL1.pressing() && l1) {
      l1 = false;
      claw.spin(fwd, 5, volt);
    }
    // if (controller_1.ButtonL1.pressing() && !l1 && clawState == 1) {
    //   l1 = true;
    //   clawRotator.spin(fwd, -4, volt);
    //   claw.spin(fwd, 0, volt);
    //   clawState = 0;
    //   Brain.resetTimer();

    //   brainTimerOn = true;
    //   after1 = false;
    //   after2 = false;
    // } else if (!controller_1.ButtonL1.pressing() && l1) {
    //   l1 = false;
    //   clawRotator.spin(fwd, -4, volt);
    //   claw.spin(fwd, 5, volt);
    // }
    if (controller_1.ButtonR1.pressing() && !r1 && after1 == false) {
      lift.spin(fwd, -4, volt);
      r1 = true;
    } else if (!controller_1.ButtonR1.pressing() && r1) {
      r1 = false;
    }
    if (controller_1.ButtonR2.pressing() && !r2 && after1 == false) {
      r2 = true;
      lift.spin(fwd, 12, volt);
    } else if (!controller_1.ButtonR2.pressing() && r2) {
      r2 = false;
    }
    if (!controller_1.ButtonR1.pressing() && !controller_1.ButtonR2.pressing() && after1 == false) {
      lift.stop(hold);
    }

    // Toggle control: Button X = Toggle on/off. Hold it to override the toggle and run the motor. If the optical sensor detects the wrong color, it will stop the motor.
    if(!controller_1.ButtonL2.pressing() && l2){
      l2 = false;
    } else if (controller_1.ButtonL2.pressing() && !toggleOn && !l2) {
      toggle.spin(reverse, 12, volt);
      toggleOn = true;
      l2 = true;
    } else if ((controller_1.ButtonL2.pressing() && toggleOn && !l2)) {
      toggle.stop(hold);
      toggleOn = false;
      l2 = true;
    } else if (toggleOptical.isNearObject() && toggleOptical.value() > 180 && toggleOptical.value() < 210 && toggleOn && selectedRollerColor == static_cast<int>(RollerColor::Blue) && !l2) {
      toggle.stop(hold);
      toggleOn = false;
      l2 = false;
    } else if (toggleOptical.isNearObject() && toggleOptical.value() > 0 && toggleOptical.value() < 30 && toggleOn && selectedRollerColor == static_cast<int>(RollerColor::Red) && !l2) {
      toggle.stop(hold);
      toggleOn = false;
      l2 = false;
    }
    wait(15, msec);
    //red 0-30
    //yellow 60-90
    //blue 180-210
    // if(!controller_1.ButtonY.pressing() && buttonY){
    //   buttonY = false;
    // } else if (controller_1.ButtonY.pressing() && !toggleOn && !buttonY) {
    //   toggle.spin(reverse, 9, volt);
    //   toggleOn = true;
    //   buttonY = true;
    // } else if ((controller_1.ButtonY.pressing() && toggleOn && !buttonY)) {
    //   toggle.stop(hold);
    //   toggleOn = false;
    //   buttonY = true;
    // }
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
void drawRollerColorSelector() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(mono20);
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Select toggle color");

  const int button_y = 70;
  const int button_width = 140;
  const int button_height = 90;
  const int button_gap = 20;
  const RollerColor colors[] = {
      RollerColor::Red, RollerColor::Blue, RollerColor::Yellow};
  const color fills[] = {red, blue, yellow};
  const char* labels[] = {"RED", "BLUE", "YELLOW"};

  for (int i = 0; i < 3; ++i) {
    const int x = 10 + i * (button_width + button_gap);
    Brain.Screen.setFillColor(fills[i]);
    Brain.Screen.setPenColor(colors[i] == static_cast<RollerColor>(selectedRollerColor) ? white:black);
    Brain.Screen.drawRectangle(x, button_y, button_width, button_height);
    Brain.Screen.setCursor(button_y / 20 + 1, x / 10 + 2);
    Brain.Screen.print("%s", labels[i]);
  }

  Brain.Screen.setPenColor(white);
  Brain.Screen.setCursor(11, 2);
  Brain.Screen.print("Selected color: %d", selectedRollerColor);
}
void rollerColorSelectorLoop() {
  bool was_pressed = false;
  while (!Competition.isEnabled()) {
    const bool pressed = Brain.Screen.pressing();
    if (pressed && !was_pressed) {
      const int x = Brain.Screen.xPosition();
      const int y = Brain.Screen.yPosition();
      const int button_y = 70;

      if (y >= button_y && y <= button_y + 90) {
        if (x >= 10 && x <= 150) {
          selectedRollerColor = static_cast<int>(RollerColor::Red);
        } else if (x >= 170 && x <= 310) {
          selectedRollerColor = static_cast<int>(RollerColor::Blue);
        } else if (x >= 330 && x <= 470) {
          selectedRollerColor = static_cast<int>(RollerColor::Yellow);
        }
        Brain.Screen.setCursor(11, 2);
  Brain.Screen.print("Selected color: %d", selectedRollerColor);
        // setRollerTargetColor(
        // static_cast<RollerColor>(selected_roller_color));
        // drawRollerColorSelector();
      }
    }
    was_pressed = pressed;
    
    wait(20, msec);
  }
}
void runPreAutonomous() {
    // Initializing   mvc  Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //startSubsystems();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }
  // double current_heading = inertial_sensor.heading();
  // Brain.Screen.print(current_heading);
  drawRollerColorSelector();
  thread rollerColorSelectorThread = thread(rollerColorSelectorLoop);
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