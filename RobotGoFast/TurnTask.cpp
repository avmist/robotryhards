#include "TurnTask.h"

TurnTask::TurnTask(Task * parent, float deg, String name) : Task(TURN, parent, name), pid(1.0, 1.0, 1.0, 4.0, 4.0) {
  this->deg = deg;
}

TurnTask::TurnTask(Task * mom, Task * dad, float deg, String name) : Task(TURN, mom, dad, name), pid(1.0, 1.0, 1.0, 4.0, 4.0) {
  this->deg = deg;
}

bool TurnTask::update() {

  if(TurnTask::between(finalYaw - initialYaw, 0, 180)) {
    leftMotor.set(8, Stepper::FORWARD);
    rightMotor.set(8, Stepper::BACKWARD);
  } else {
    leftMotor.set(8, Stepper::BACKWARD);
    rightMotor.set(8, Stepper::FORWARD);
  }

  if(TurnTask::between(imu.yaw, (finalYaw - 2) % 360), (finalYaw + 2) % 360)) {
    leftMotor.stop();
    rightMotor.stop();
    return true;
  }
  
  return false;

}

void TurnTask::init() {
  
  Task::init();

  leftMotor.resetCount();
  rightMotor.resetCount();

  // TODO capture IMU yaw
  initialYaw = imu.yaw;

  finalYaw = (initialYaw + deg) % 360;

  // Calculate distance to move
  double distnce = PI * Stepper::wheelSpacing * (abs(deg) / 360.f);

  Serial.print("Arc distance to travel: ");
  Serial.print(distnce);
  Serial.print(" in.\n");

  // Calculate degree per step
  double degPerStep = 360.f / (Stepper::stepsPerRevolution * Stepper::microsteps);

  Serial.print("Deg per step: ");
  Serial.print(degPerStep);
  Serial.print(" deg.\n");

  // Calculate distance per step
  double distPerStep = PI * Stepper::wheelDiameter * (degPerStep / 360.f);

  Serial.print("Dist per step: ");
  Serial.print(distPerStep);
  Serial.print(" in.\n");
  
  steps = distnce / distPerStep;
  
  Serial.print("Need to step: ");
  Serial.print(steps);
  Serial.print(" times.\n");
  
}

bool TurnTask::between(float value, float low, float high) {

  if(low > high) {
    return (value >= high && value <= low);
  } else {
    return (value >= low && value <= high);
  }

}

