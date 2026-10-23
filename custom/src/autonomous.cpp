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
void exampleAuton2() {

}
void Pin3Quad1() {
 x_pos = 4.25;
  y_pos = -62;
  clawRotator.spin(fwd, 4, volt);
  claw.spin(fwd, 5, volt);
  //toggle 
  toggle.spin(reverse, 12, volt);
  while(toggleOptical.value() < 5 || toggleOptical.value() > 45)
  {
    vex::task::sleep(10);
  }
  wait(200, msec);
  toggle.stop(hold);
  lift.setVelocity(80, pct);
  driveTo(8, 1000, true, 8);
  lift.spinToPosition(100 , deg, false);
  turnToPoint(24, -47, 1, 1000);
  moveToPoint(24, -47, 1, 1250, true, 6);
  lift.spin(fwd, -2, volt);
  wait(300, msec);
  lift.stop(hold);
  clawRotator.spin(fwd, -4, volt);
  claw.spin(fwd, 0, volt);
  wait(200, msec);
  driveTo(-4, 500, false, 8);
  moveToPoint(6.5, -52, -1, 1000, true);
  turnToPoint(24, -30, 1, 1000);
  claw.spin(fwd, 5, volt);
  lift.spin(fwd, -2, volt);
  driveTo(2.5, 1000, false, 8);
  wait(500,msec);
  driveTo(1, 1000, false, 4);
  Brain.resetTimer();
  while(clawDetect.value() > 90 && Brain.timer(msec) < 700){
    vex::task::sleep(10);}
  //driveTo(-1, 100, false, 6);
  //moveToPoint(-23.5, -24, 1, 1450, false, 4);
  //Finish Grabbing Game Piece
  clawRotator.spin(fwd, 4, volt);
  lift.stop(hold);
  wait(200, msec);
  stopChassis(hold);
  wait(100, msec);
  //turnToPoint(-24, -48, 1, 2000);
  turnToAngle(-185, 800);
  lift.setVelocity(80, pct);
  lift.spinToPosition(190, deg, false);
  //moveToPoint(-24, -48, 1, 1100, true, 4);
  driveTo(80, 950, false, 4);
  lift.spin(fwd, -4, volt);
  wait(400, msec);
  lift.stop(hold);
  stopChassis(brake);
  clawRotator.spin(fwd, -4, volt);  
  claw.spin(fwd, 0, volt);
  //Finish Stacking Game Piece
  // resetPositionFront();
  // resetPositionRight();
  driveTo(-13.5, 1500, true, 8);
  turnToPoint(47, -50, 1, 1000);
  claw.spin(fwd, 5, volt);
  lift.spin(fwd, -4, volt);
  moveToPoint(35.5, -39, 1, 1450, false, 8);
  turnToPoint(48, -50, 1, 300);
  driveTo(1, 1000, false, 6);
  Brain.resetTimer();
  while(clawDetect.value() > 90 && Brain.timer(msec) < 700)
  {
    vex::task::sleep(10);
  }
  //moveToPoint(-23.5, -24, 1, 1450, false, 4);
  clawRotator.spin(fwd, 4, volt);
  lift.stop(hold);
  wait(200, msec);
  stopChassis(hold);
  wait(400, msec);
  turnToAngle(-99, 700);
  lift.setVelocity(80, pct);
  lift.spinToPosition(300, deg, false);
  //moveToPoint(-24, -48, 1, 1100, true, 4);
  driveTo(80, 1000, false, 4);
  lift.spin(fwd, -4, volt);
  wait(300, msec);
  stopChassis(brake);
  lift.stop(hold);
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
  claw.spin(fwd, 5, volt);
  //toggle 
  toggle.spin(reverse, 12, volt);
  while(toggleOptical.value() < 5 || toggleOptical.value() > 45)
  {
    vex::task::sleep(10);
  }
  wait(200, msec);;
  toggle.stop(hold);
  lift.setVelocity(80, pct);
  lift.spinToPosition(130 , deg, false);
  turnToPoint(-24, -47, 1, 500);
  moveToPoint(-24, -47, 1, 800, false, 6);
  driveChassis(1,1);
  lift.spin(fwd, -2, volt);
  vex::wait(300, msec);
  lift.stop(hold);
  clawRotator.spin(fwd, -4, volt);
  claw.spin(fwd, 0, volt);
  vex::wait(200, msec);
  moveToPoint(-5, -52, -1, 1000, true, 10);
  turnToPoint(-18, -37, 1, 800);
  claw.spin(fwd, 5, volt);
  lift.spin(fwd, -2, volt);
  moveToPoint(-18,-37,1,1000,true,8);
  driveChassis(4,4);
  Brain.resetTimer();
  while(clawDetect.value() > 90 && Brain.timer(msec) < 700){
    vex::task::sleep(10);
  }
  wait(200,msec);
  clawRotator.spin(fwd, 4, volt);
  lift.stop(hold);
  lift.setVelocity(80, pct);
  lift.spinToPosition(190, deg, false);
  turnToPoint(-20, -40, 1, 1000);
  moveToPoint(-20,-40,1,1000,false,8);
  driveChassis(4,4);
  wait(200,msec);
  lift.spin(fwd, -4, volt);
  wait(400, msec);
  lift.stop(hold);
  stopChassis(brake);
  clawRotator.spin(fwd, -4, volt);  
  claw.spin(fwd, 0, volt);
  

  
  /**
  //driveTo(-1, 100, false, 6);
  //moveToPoint(-23.5, -24, 1, 1450, false, 4);
  //Finish Grabbing Game Piece
  clawRotator.spin(fwd, 4, volt);
  lift.stop(hold);
  wait(200, msec);
  stopChassis(hold);
  wait(100, msec);
  //turnToPoint(-24, -48, 1, 2000);
  turnToAngle(185, 800);
  lift.setVelocity(80, pct);
  lift.spinToPosition(190, deg, false);
  //moveToPoint(-24, -48, 1, 1100, true, 4);
  driveTo(80, 950, false, 4);
  lift.spin(fwd, -4, volt);
  wait(400, msec);
  lift.stop(hold);
  stopChassis(brake);
  clawRotator.spin(fwd, -4, volt);  
  claw.spin(fwd, 0, volt);
  //Finish Stacking Game Piece
  // resetPositionFront();
  // resetPositionRight();
  driveTo(-13.5, 1500, true, 8);
  turnToPoint(-47, -50, 1, 1000);
  claw.spin(fwd, 5, volt);
  lift.spin(fwd, -4, volt);
  moveToPoint(-35.5, -39, 1, 1450, false, 8);
  turnToPoint(-48, -50, 1, 300);
  driveTo(1, 1000, false, 6);
  Brain.resetTimer();
  while(clawDetect.value() > 90 && Brain.timer(msec) < 700)
  {
    vex::task::sleep(10);
  }
  //moveToPoint(-23.5, -24, 1, 1450, false, 4);
  clawRotator.spin(fwd, 4, volt);
  lift.stop(hold);
  wait(200, msec);
  stopChassis(hold);
  wait(400, msec);
  turnToAngle(99, 700);
  lift.setVelocity(80, pct);
  lift.spinToPosition(300, deg, false);
  //moveToPoint(-24, -48, 1, 1100, true, 4);
  driveTo(80, 1000, false, 4);
  lift.spin(fwd, -4, volt);
  wait(300, msec);
  stopChassis(brake);
  lift.stop(hold);
  clawRotator.spin(fwd, -4, volt);
  claw.spin(fwd, 0, volt);
  //Finish Stacking Game Piece
  // moveToPoint(24, 24, 1, 2000, false);
  // moveToPoint(48, 48, 1, 2000, true);
  // moveToPoint(24, 24, -1, 2000, true);
  // moveToPoint(0, 0, 1, 2000, true);
  // correct_angle = 0;
  // driveTo(24, 2000, false, 8);
  // turnToAngle(90, 800,      false);
  // turnToAngle(180, 800, true);
  */
  
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