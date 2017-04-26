#include "TurnTask.h"

TurnTask::TurnTask(Task * parent, double deg, String name) : Task(TURN, parent, name), pid(1.0, 1.0, 1.0, 4.0, 4.0) {
  this->deg = deg;
}

TurnTask::TurnTask(Task * mom, Task * dad, double deg, String name) : Task(TURN, mom, dad, name), pid(1.0, 1.0, 1.0, 4.0, 4.0) {
  this->deg = deg;
}

bool TurnTask::update() {

  if(deg > 0) {
    leftMotor.set(8, Stepper::FORWARD);
    rightMotor.set(8, Stepper::BACKWARD);
  } else {
    leftMotor.set(8, Stepper::BACKWARD);
    rightMotor.set(8, Stepper::FORWARD);
  }

  if(leftMotor.getCount() > steps) {
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

