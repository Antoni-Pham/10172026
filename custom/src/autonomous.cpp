#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "../include/subsystems.h"
#include "../include/robot-config.h"

#include "motor-control.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }

namespace {

// Wrap any motion sequence in this callback and pass its arguments as data.
typedef void (*AutonomousMovement)(void*);

volatile bool movement_finished = false;
AutonomousMovement autonomous_movement = 0;
void* autonomous_movement_data = 0;

struct DriveMovement {
  double distance_inches;
  int duration_msec;
  bool exit;
  double speed;
};

struct PointMovement {
  double x;
  double y;
  int direction;
  int duration_msec;
  bool exit;
  double speed;
  bool overturn;
};

struct PoseMovement {
  double x;
  double y;
  int direction;
  double target_angle;
  double dlead;
  int duration_msec;
  bool exit;
  double speed;
  bool overturn;
};

void runDriveMovement(void* data) {
  DriveMovement* movement = static_cast<DriveMovement*>(data);
  driveTo(movement->distance_inches, movement->duration_msec, movement->exit, movement->speed);
}

void runPointMovement(void* data) {
  PointMovement* movement = static_cast<PointMovement*>(data);
  moveToPoint(movement->x, movement->y, movement->direction,
              movement->duration_msec, movement->exit, movement->speed, movement->overturn);
}

void runPoseMovement(void* data) {
  PoseMovement* movement = static_cast<PoseMovement*>(data);
  boomerang(movement->x, movement->y, movement->direction, movement->target_angle, movement->dlead, movement->duration_msec, movement->exit, movement->speed, movement->overturn);
}

void runAutonomousMovement() {
  autonomous_movement(autonomous_movement_data);
  movement_finished = true;
}

void driveToDist(vex::distance& dist, double target_inches, double speed, bool forward) {
  if (!std::isfinite(target_inches) || target_inches <= 0 ||
      !std::isfinite(speed) || speed <= 0) {
    logger.error("Invalid driveToDist parameters");
    stopChassis(hold);
    return;
  }

  const double tolerance = 3;
  const double time_limit_msec = 5000;
  const double start_time = Brain.timer(msec);

  while (Brain.timer(msec) - start_time < time_limit_msec) {
    if (!dist.isObjectDetected()) {
      logger.error("driveToDist lost distance sensor target");
      stopChassis(brake);
      return;
    }

    const double current_distance = dist.objectDistance(vex::inches);
    if (!std::isfinite(current_distance) || current_distance <= 0) {
      logger.error("driveToDist received invalid distance reading");
      stopChassis(brake);
      return;
    }

    if (fabs(target_inches - current_distance) <= tolerance) {
      stopChassis(brake);
      vex::wait(50, msec);
      return;
    }

    const double output = forward ? speed : -speed;
    driveChassis(output, output);
    vex::wait(10, msec);
  }

  logger.error("driveToDist timed out at %.2f inches",
               dist.objectDistance(vex::inches));
}
bool runMovementWhileWaitingForGamePiece(AutonomousMovement movement,
                                         void* movement_data,
                                         int acquisition_timeout_msec) {
  if (isGamePieceAcquired()) return true;

  autonomous_movement = movement;
  autonomous_movement_data = movement_data;
  movement_finished = false;
  thread movement_task = thread(runAutonomousMovement);

  const double start_time = Brain.timer(msec);
  bool acquired = false;
  while (Brain.timer(msec) - start_time < acquisition_timeout_msec) {
    if (isGamePieceAcquired()) {
      acquired = true;
      break;
    }
    wait(10, msec);
  }
  if (!acquired) acquired = isGamePieceAcquired();
  if (!movement_finished) {
    movement_task.interrupt();
    stopChassis(hold);
  }
  return acquired;
}

}  // namespace

void WallAuton(){
  x_pos = -4.25;
  y_pos = -62;
  clawRotator.spin(fwd, 4, volt);
  claw.spin(fwd, 5, volt);
  //toggle 
  toggle.spin(reverse, 12, volt);
  while(toggleOptical.value() < 5 || toggleOptical.value() > 45){
    vex::task::sleep(10);
  }
  wait(200, msec);
  toggle.stop(hold);
  lift.setVelocity(80, pct);
  driveTo(14, 1000, true, 8);
  lift.spinToPosition(130, deg, false);
  turnToPoint(-24, -47, 1, 1000);
  moveToPoint(-24, -47, 1, 1450, true, 4);
  lift.spin(fwd, -2, volt);
  wait(300, msec);
  lift.stop(hold);
  clawRotator.spin(fwd, -4, volt);
  claw.spin(fwd, 0, volt);
  wait(200, msec);
  driveTo(-10, 1000, true, 10);
  clawRotator.spin(fwd, 4, volt);
  claw.spin(fwd, 5, volt);
  lift.spin(fwd, -2, volt);
  turnToPoint(-24, -68, 1, 1000);
  driveTo(10, 1500, false, 10);
   while(clawDetect.value() > 90 && Brain.timer(msec) < 700){
    vex::task::sleep(10);
  }
}

void exampleAuton() {
  DriveMovement first_approach = {24, 2000};
  PointMovement second_approach = {18, 18, 1, 2000, false, 8, false};

  setRollerTargetColor(RollerColor::Red);
  if (runMovementWhileWaitingForGamePiece(
          runDriveMovement, &first_approach, 2500)) {
    scoreLiftAutonomous();  // First completed score targets 10 inches.
  }
  turnToAngle(90, 2000);
  setRollerTargetColor(RollerColor::Yellow);
  if (runMovementWhileWaitingForGamePiece(
          runPointMovement, &second_approach, 2500)) {
    scoreLiftAutonomous();  // Second completed score targets 20 inches.
  }
  turnToAngle(180, 2000);
  setRollerTargetColor(RollerColor::Blue);
  setLiftTargetHeight(15.0);  // Demonstrate a custom lift target in inches.
  wait(1000, msec);
  driveTo(-42, 3000);
}

void exampleAuton2(){
  resetPositionBack();
  resetPositionRight();
}

volatile int i = 0;

void Skills() {
  Pin3Quad1(1);
  claw.spin(fwd, 5, volt);
  lift.spin(fwd, -4, volt);
  driveChassis(-3,-3);
  wait(300,msec);
  swing(270,-1,2000,true,12);
  resetPositionBack();
  wait(10,msec);
  moveToPoint(58,y_pos,-1,2000,true,8);
  turnToAngle(180,2000,true,12);
  for (i = 0; i < 1; i++) { 
    resetPositionLeft();
    resetPositionFront();
    // moveToPoint(60, -55, 1, 1000, false,6);
    driveChassis(5,5);
    Brain.resetTimer();
    int time = Brain.timer(msec);
    while(clawDetect.value() > 39 && (Brain.timer(msec)- time) < 1500){
      vex::task::sleep(10);
    }
    wait(50,msec);
    driveChassis(-3,-3);
    wait(175,msec);
    clawRotator.spin(fwd,3,volt);
    wait(250,msec);
    vex::thread([](){
      lift.setVelocity(80, pct);
      if (i == 0){
          lift.spinToPosition(390, deg, false);
          vex::wait(500, msec);
      } else if (i == 1) {
          lift.spinToPosition(475, deg, false);
          vex::wait(500, msec);
      } return 0;
    });
    moveToPoint(38, -53, 1, 1000, true);
    driveToHeading(17, 275, 700, false, 4.8);
    driveToHeading(10, 275, 400, false, 2);
    wait(350, msec);
    lift.spin(fwd, -3.5, volt);
    wait(400, msec);
    lift.stop(hold);
    stopChassis(brake);
    clawRotator.spin(fwd, -4, volt);  
    claw.spin(fwd, 0, volt);
    driveChassis(-5,-5);
    wait(200,msec);
    turnToAngle(270,1000,true,8);
    Brain.resetTimer();
    claw.spin(fwd, 5, volt);
    lift.spin(fwd, -4, volt);
   driveChassis(-3,-3);
    wait(300,msec);
    swing(270,-1,2000,true,12);
    resetPositionBack();
    wait(10,msec);
    moveToPoint(58,y_pos,-1,2000,true,8);
    turnToAngle(180,2000,true,12);
  }
  MatchloadFar();
  ToggleFar();
}

void MatchloadFar() {

  //turnToAngle(180, 3000, true, 12);
  
    driveChassis(5,5);
  for(i = 0; i < 4; i++){
    claw.spin(fwd, 5, volt);
    clawRotator.spin(fwd,3,volt);
    lift.spin(fwd, -4, volt);
    if (i == 0) {
      driveChassis(5,5);
      lift.setVelocity(80,percent);
      lift.spinToPosition(15,degrees);
    }
    wait(100,msec);
    resetPositionLeft();
    resetPositionFront();
    wait(10,msec);
    Brain.resetTimer();
    if (i == 0) {
    
       while(clawDetect.value() > 130 && Brain.timer(msec) < 2000){
      vex::task::sleep(10);
        vex::wait(300,msec);
        driveChassis(4,4);
      
      }
  }
    if (i > 0) {
    //boomerang(58,-54,1,180,0.2,2000,true,8);
    driveChassis(5,5);
    clawRotator.spin(fwd,-3,volt);
    while(clawDetect.value() > 39 && Brain.timer(msec) < 2000){
      vex::task::sleep(10);
    }
     }
    wait(50,msec);
    driveChassis(-2.3,-2.3);
    wait(175,msec);
    clawRotator.spin(fwd,3,volt);
    wait(300,msec);
    lift.setVelocity(60, pct);
    lift.spinToPosition(130 + i*70 , deg, false);
    if (i==4){
      lift.spinToPosition(510 , deg, false);
    }
    moveToPoint(51, -44, -1, 2000, true, 7);
    //moveToPoint(50, -48, -1, 1000, true, 8);
    //turnToPointTwo(46, -24, -1, 1000, 7);
    turnToPoint(46,-24,1,1000);
    driveToHeading(20, 355, 1000, true, 7);
    resetPositionBack();
    resetPositionRight();
    if (i==0){
      driveChassis(-1, -1);
      lift.spin(fwd, -6, volt);
      wait(160,msec);
      stopChassis(hold);
      wait(300,msec);
      lift.stop(hold);
      clawRotator.spin(fwd, -4, volt);
      claw.spin(fwd, -4, volt);
      driveChassis(-5, -5);
      wait(150,msec);
    } else {
      lift.spin(fwd, -3, volt);
      vex::wait(500, msec);
      lift.stop(hold);
      clawRotator.spin(fwd, -4, volt);
      claw.spin(fwd, 3, volt);
      if (i==4){
        claw.spin(fwd, -3, volt);
    }
    }

    // driveToDist(back_sensor, 19, 7, false);
    // turnToAngle(-90, 900, true, 7);
    // driveToDist(back_sensor, 9, 7, false);
    // turnToAngle(180, 900, true, 7);
    driveChassis(-5,-5);
    wait(200,msec);
    boomerang(59.5, -48, -1, -10, 0.2, 2000,true,8);
    claw.spin(fwd, 5, volt);
    lift.spin(fwd, -3, volt);
    turnToAngle(180,1000,true,8);
  }
    //boomerang(44, -24, 1, , 0.2, 1000, true, 7, false);
    //moveToPoint(55, -50, -1, 1000, false, 10, false);
    //turnToAngle(180, 1000, true, 7);
    //driveToHeading(40, -10, 1000, true, 7);
    

    /*
    turnToPoint(48, -24, 1000, true);
    driveTo(15, 1000, false, 6);
    // driveToHeading(0, 275, 700, false, 6);
    // driveToHeading(-7, 275, 300, false, 3);
    lift.spin(fwd, -4, volt);
    wait(500, msec);
    lift.stop(hold);
    stopChassis(brake);
    clawRotator.spin(fwd, -4, volt);  
    claw.spin(fwd, 0, volt);
    driveChassis(-5,-5);
    wait(200,msec);
    turnToAngle(270,1000,true,8);
    Brain.resetTimer();
    claw.spin(fwd, 5, volt);
    lift.spin(fwd, -8, volt);
    driveToDist(back_sensor, 10, 8, false);
    turnToAngle(180, 1000, true, 8);
    clawRotator.spin(fwd,3,volt);
  for (i = 0; i < 3; i++) { 
    resetPositionLeft();
    resetPositionFront();
    // moveToPoint(60, -55, 1, 1000, false,6);
    driveChassis(5,5);
    Brain.resetTimer();
    while(clawDetect.value() > 39 && Brain.timer(msec) < 500){
      vex::task::sleep(10);
    }
    wait(50,msec);
    driveChassis(-3,-3);
    wait(450,msec);
    vex::thread([](){
      lift.setVelocity(80, pct);
      if (i == 0){
          lift.spinToPosition(190, deg, false);
          vex::wait(500, msec);
      } else if (i == 1) {
          lift.spinToPosition(310, deg, false);
          vex::wait(500, msec);
      } else if (i == 2) {
          lift.spinToPosition(430, deg, false);
          vex::wait(500, msec);
      } return 0;
    });
    moveToPoint(38, -53, 1, 1000, true);
    driveToHeading(17, 275, 700, false, 6);
    driveToHeading(10, 275, 300, false, 3);
    lift.spin(fwd, -4, volt);
    wait(500, msec);
    lift.stop(hold);
    stopChassis(brake);
    clawRotator.spin(fwd, -4, volt);  
    claw.spin(fwd, 0, volt);
    driveChassis(-5,-5);
    wait(200,msec);
    turnToAngle(270,1000,true,8);
    Brain.resetTimer();
    claw.spin(fwd, 5, volt);
    lift.spin(fwd, -8, volt);
    driveToDist(back_sensor, 10, 8, false);
    turnToAngle(180, 1000, true, 8);
    clawRotator.spin(fwd,3,volt);
    */
  }

void ToggleFar() {
    resetPositionLeft();
    resetPositionFront();
    wait(10,msec);
    moveToPoint(x_pos,-6,-1,6000,true,8);
    turnToAngle(270,1000,true,8);
    driveChassis(-6,-6);
    toggle.spin(reverse, 9, volt);
     while(toggleOptical.value() < 5 || toggleOptical.value() > 45){
       vex::task::sleep(10);
     }
    toggle.stop(hold);
    moveToPoint(12,0,1,2000,true,12);  
  }

void Pin3Quad1(int color){
  x_pos = 4.25;
  y_pos = -62;
  clawRotator.spin(fwd, 4, volt);
  //claw.spin(fwd, 5, volt);
  //toggle 
  toggle.spin(reverse, 9, volt);
  if (color == 1)
  {
     while(toggleOptical.value() < 5 || toggleOptical.value() > 45){
    vex::task::sleep(10);
    }
  }
  if (color == 2)
  {
     while(toggleOptical.value() < 180 || toggleOptical.value() > 210){
    vex::task::sleep(10);
    }
  }
  toggle.stop(hold);
  lift.setVelocity(30, pct);
  lift.spinToPosition(110, deg, false);
  turnToPoint(24, -47, 1, 500);
  boomerang(24, -47, 1, 50, 0.5, 950, false, 7);
  driveChassis(-1, -1);
  lift.spin(fwd, -6, volt);
  wait(160,msec);
  stopChassis(hold);
  wait(300,msec);
  lift.stop(hold);
  clawRotator.spin(fwd, -4, volt);
  claw.spin(fwd, -4, volt);
  driveChassis(-5, -5);
  wait(150,msec);
  //lift.spinToPosition(128 , deg, true);
  moveToPoint(2, -52, -1, 1000, true, 10);
  lift.spin(fwd, -7, volt);
  turnToPoint(22, -24, 1, 800);
  claw.spin(fwd, 5, volt);
  moveToPoint(15.7, -32, 1, 2000, true, 7);
  driveChassis(4,4);
  Brain.resetTimer();
  while(clawDetect.value() > 90 && Brain.timer(msec) < 500){
    vex::task::sleep(10);
  }
  wait(200, msec);
  clawRotator.spin(fwd, 4, volt);
  lift.stop(hold);
  lift.setVelocity(80, pct);
  lift.spinToPosition(175, deg, false);
  wait(200, msec);
  // vex::thread([]
  //             {
  //   wait(200,msec);
  // clawRotator.spin(fwd, 4, volt);
  // lift.stop(hold);
  // lift.setVelocity(80, pct);
  // lift.spinToPosition(190, deg, false);
  //   return 0; });
  //turnToPoint(27, -48, 1, 900);
  driveToHeading(8, 180, 600, false, 10);
  moveToPoint(27, -48, 1, 700, true, 6);
  //moveToPoint(-40,-20,1,1000,false,8);
  driveChassis(4,4);
  wait(700,msec);
  lift.spin(fwd, -7, volt);
  wait(300, msec);
  lift.stop(hold);
  stopChassis(brake);
  clawRotator.spin(fwd, -4, volt);  
  claw.spin(fwd, 0, volt);
//Go grab 3rd Pin
  driveChassis(-5,-5);
  wait(250,msec);
  moveToPoint(24, -22.5, -1, 1000, true, 8);
  turnToPoint(50, -48, 1, 800);
  claw.spin(fwd, 5, volt);
  lift.spin(fwd, -6, volt);
  moveToPoint(39, -36, 1, 2000, false, 6.7);
  turnToPoint(50, -48, 1, 800);
  driveChassis(4, 4);
  Brain.resetTimer();
  while (clawDetect.value() > 90 && Brain.timer(msec) < 700){
    vex::task::sleep(10);
  }
  wait(100, msec);
  clawRotator.spin(fwd, 4, volt);
  //driveTo(-1, 100, false, 6);
  //moveToPoint(-23.5, -24, 1, 1450, false, 4);
  //Finish Grabbing Game Piece
  claw.spin(fwd, 4, volt);
  lift.stop(hold);
  stopChassis(hold);
  vex::thread([]{
    lift.setVelocity(80, pct);
    lift.spinToPosition(270, deg, false);
    wait(500, msec);
    return 0;
  });
  //turnToPoint(24, -51, 1, 2000);
  //turnToAngle(185, 800);
  //moveToPoint(-24, -48, 1, 1100, true, 4);
  driveToHeading(17, 275, 700, false, 10);
  driveToHeading(14, 275, 1000, false, 5);
  lift.spin(fwd, -5.5, volt);
  wait(200, msec);
  clawRotator.spin(fwd, -4, volt); 
  wait(200, msec);
  lift.stop(hold);
  stopChassis(brake);
  clawRotator.spin(fwd, -4, volt);  
  claw.spin(fwd, 0, volt);
  //Finish Stacking Game Piece
}

void Pin3Quad2() {
  // controller_1.Screen.clearScreen();
  // controller_1.Screen.setCursor(1,1);
  // while (true){
  //    controller_1.Screen.clearScreen();
  //   controller_1.Screen.print("Odom: %.2f, %.2f, %.2f", x_pos, y_pos, getInertialHeading());
  //    controller_1.Screen.setCursor(1,1);
  //   wait(300, msec);
  // }
  // return;
  // resetPositionBack();
  // resetPositionLeft();
  x_pos = -4.25;
  y_pos = -62;
  clawRotator.spin(fwd, 4, volt);
  //claw.spin(fwd, 5, volt);
  //toggle 
  toggle.spin(reverse, 12, volt);
  while(toggleOptical.value() < 5 || toggleOptical.value() > 45)
  {
    vex::task::sleep(10);
  }
  wait(200, msec);;
  toggle.stop(hold);
  lift.setVelocity(60, pct);
  lift.spinToPosition(125 , deg, false);
  wait(200,msec);
  turnToPoint(-27, -47, 1, 500);
  moveToPoint(-27, -47, 1, 950, false, 6);
  driveChassis(5,5);
  lift.spin(fwd, -3, volt);
  vex::wait(450, msec);
  lift.stop(hold);
  clawRotator.spin(fwd, -4, volt);
  claw.spin(fwd, 3, volt);
  vex::wait(350, msec);
  //lift.spinToPosition(128 , deg, true);
  moveToPoint(-5, -52, -1, 1000, true, 10);
  lift.spin(fwd, -2, volt);
  turnToPoint(-15, -40, 1, 800);
  claw.spin(fwd, 5, volt);
  moveToPoint(-15, -40, 1, 2000, true, 8);
  driveChassis(4,4);
  Brain.resetTimer();
  while(clawDetect.value() > 90 && Brain.timer(msec) < 500){
    vex::task::sleep(10);
  }
  wait(200, msec);
  clawRotator.spin(fwd, 4, volt);
  lift.stop(hold);
  lift.setVelocity(80, pct);
  lift.spinToPosition(190, deg, false);
  // vex::thread([]
  //             {
  //   wait(200,msec);
  // clawRotator.spin(fwd, 4, volt);
  // lift.stop(hold);
  // lift.setVelocity(80, pct);
  // lift.spinToPosition(190, deg, false);
  //   return 0; });
  turnToPoint(-23, -49, 1, 900);
  moveToPoint(-23, -49, 1, 950, true, 6);
  //moveToPoint(-40,-20,1,1000,false,8);
  driveChassis(4,4);
  wait(400,msec);
  lift.spin(fwd, -4, volt);
  wait(400, msec);
  lift.stop(hold);
  stopChassis(brake);
  clawRotator.spin(fwd, -4, volt);  
  claw.spin(fwd, 0, volt);
//Go grab 3rd Pin
  moveToPoint(-23, -30, -1, 1000, true, 8);
  turnToPoint(-35, -44, 1, 800);
  claw.spin(fwd, 5, volt);
  lift.spin(fwd, -3, volt);
  moveToPoint(-35, -44, 1, 2000, false, 8);
  turnToPoint(-47, -56, 1, 800);
  driveChassis(4, 4);
  Brain.resetTimer();
  while (clawDetect.value() > 90 && Brain.timer(msec) < 700){
    vex::task::sleep(10);
  }
  wait(550, msec);
  clawRotator.spin(fwd, 4, volt);
  //driveTo(-1, 100, false, 6);
  //moveToPoint(-23.5, -24, 1, 1450, false, 4);
  //Finish Grabbing Game Piece
  claw.spin(fwd, 4, volt);
  lift.stop(hold);
  stopChassis(hold);
  lift.setVelocity(80, pct);
  lift.spinToPosition(310, deg, false);
  turnToPoint(0, -54, 1, 2000);
  //turnToAngle(185, 800);
  //moveToPoint(-24, -48, 1, 1100, true, 4);
  driveTo(80, 600, false, 8);
  driveTo(80, 100, false, 4);
  lift.spin(fwd, -4, volt);
  wait(500, msec);
  lift.stop(hold);
  stopChassis(brake);
  clawRotator.spin(fwd, -4, volt);  
  claw.spin(fwd, 0, volt);
  //Finish Stacking Game Piece



}



/*
 * armPID
 * Runs a single PID update for the arm motor to reach the specified target position.
 * - arm_target: Desired arm position (degrees).
 
void armPID(double arm_target) {
  /*
  PID pidarm = PID(0.1, 0, 0.5); // Initialize PID controller for arm
  pidarm.setTarget(arm_target);   // Set target position
  pidarm.setIntegralMax(0);  
  pidarm.setIntegralRange(1);
  pidarm.setSmallBigErrorTolerance(1, 1);
  pidarm.setSmallBigErrorDuration(0, 0);
  pidarm.setDerivativeTolerance(100);
  pidarm.setArrive(true);
  arm_motor.spin(fwd, pidarm.update(arm_motor.position(deg)), volt); // Apply PID output to arm motor
}

/*
 * armPIDLoop
 * Continuously runs the arm PID control in a separate thread, keeping the arm at the target position.
 
void armPIDLoop() {
  while(true) {
    armPID(arm_pid_target); // Continuously update arm position
    wait(10, msec);
  }
}

/*
 * rushClamp
 * Waits until the clamp distance sensor detects an object within 85mm, then closes the claw and lowers the rush arm.
 * Used for quickly grabbing a mobile goal at the start of autonomous.
 
void rushClamp() {
  while(clamp_distance.objectDistance(mm) > 85) { // Wait for object to be close enough
    wait(10, msec);
  }
  claw.set(true);        // Close the claw to grab the goal
  rush_arm.set(false);   // Lower the rush arm
}

/*
 * intakeThread
 * Runs the intake until an object is detected by the optical or distance sensor, then stops the intake.
 * Used for picking up rings or other objects during autonomous.
 
void intakeThread(){
  optical_sensor.setLight(ledState::on);      // Turn on optical sensor light
  optical_sensor.setLightPower(100);          // Set light power to max
  while(!optical_sensor.isNearObject() && intake_distance.objectDistance(mm) > 50){
    wait(10, msec);                           // Wait until object is detected
  }
  intake_motor.stop(hold);                    // Stop intake motor and hold
}

/*
 * redGoalRush
 * 2024-2025 World Championship runner-up(1698V) autonomous routine.
 * This routine executes a complex sequence to rush, grab, and score mobile goals and rings.
 * It uses multiple threads for simultaneous arm, clamp, and intake control.
 
void redGoalRush() {
  arm_motor.setPosition(arm_load_target, deg);         // Set arm to load position
  correct_angle = inertial_sensor.rotation();          // Sync correct_angle with inertial sensor
  arm_pid_target = arm_store_target;                   // Set arm PID target to store position

  thread al = thread(armPIDLoop);                      // Start arm PID loop in a thread
  thread rc = thread(rushClamp);                       // Start clamp routine in a thread
  intake_motor.spin(fwd, 12, volt);                    // Start intake motor at full speed
  thread it = thread(intakeThread);                    // Start intake sensor thread
  rush_arm.set(true);                                  // Lower rush arm

  driveTo(33, 1100, true);                             // Drive forward to first goal
  moveToPoint(-2, 10, -1, 15000, false);               // Pull the goal back
  stopChassis(hold);                                   // Stop chassis and hold position

  rc.interrupt();                                      // Stop clamp thread (goal should be clamped)
  rush_arm.set(true);                                  // Lower rush arm again (ensure down)
  claw.set(false);                                     // Open claw to release goal
  wait(100, msec);                                     // Brief pause

  correct_angle = normalizeTarget(-20);                // Adjust target heading for next maneuver
  driveTo(3, 800, true, 8);                            // Drive forward slightly
  driveTo(-5, 1000, true);                             // Back up

  rush_arm.set(false);                                 // Raise rush arm
  wait(200, msec);                                     // Wait for arm to raise

  turnToAngle(-90, 800, false);                        // Turn to face the goal backwards
  moveToPoint(0, 26, -1, 2000, false, 6);              // Move backwards into the goal
  driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
  mogo_mech.set(true);                                 // Clamp mobile goal
  wait(100, msec);                                     // Wait for clamp

  it.interrupt();                                      // Stop intake thread (ring should be collected)
  intake_motor.spin(fwd, 12, volt);                    // Restart intake

  moveToPoint(1, 7, 1, 2000, true);                    // Move near corner to drop goal
  turnToAngle(-90, 350, true);                         // Turn to drop goal
  mogo_mech.set(false);                                // Release mobile goal clamp
  driveChassis(-4, 4);                                 // Turn a bit to align with next target
  wait(300, msec);                                     // Wait for spin

  intake_motor.spin(fwd, -12, volt);                   // Reverse intake to push disc in front away
  moveToPoint(-13, -4, 1, 1500, false, 10);            // Move forward to push disc out of the way
  turnToAngle(180, 800, false);                        // Turn to clamp goal
  intake_motor.spin(fwd, 0, volt);                     // Stop intake

  moveToPoint(-31, 26, -1, 2000, false, 6);            // Move backwards into the next goal
  driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
  mogo_mech.set(true);                                 // Clamp mobile goal
  wait(100, msec);                                     // Wait for clamp

  turnToAngle(145, 300, true);                         // Turn to face corner
  moveToPoint(-4, -3, 1, 2000, false);                 // Move to corner
  intake_motor.spin(fwd, 12, volt);                    // Start intake

  correct_angle = normalizeTarget(135);                // Update heading for next maneuver
  driveTo(1000, 1500, false, 4);                       // Drive forward infinitely until timeout
  driveTo(-13, 2000, true, 6);                         // Back up
  driveTo(10, 2500, true, 3);                          // Drive forward to intake second corner ring

  wait(200, msec);                                     // Brief wait for intake

  moveToPoint(-11, 6, -1, 2000, false, 10);            // Move backward out of the corner
  turnToAngle(45, 400, true);                          // Turn to align for wallstake

  al.interrupt();                                      // Stop arm PID thread
  arm_pid_target = arm_score_target - 100;             // Set arm to scoring position
  thread al2 = thread(armPIDLoop);                     // Start new arm PID thread

  moveToPoint(12, 34, 1, 1700, true, 8);               // Move forward to final wallstake scoring position

  al2.interrupt();                                     // Stop arm PID thread
  arm_motor.spin(fwd, 1, volt);                        // Spin arm forward slightly

  turnToAngle(40, 200);                                // Final turn for alignment
  driveChassis(1, 1);                                  // Slow drive forward
}
*/