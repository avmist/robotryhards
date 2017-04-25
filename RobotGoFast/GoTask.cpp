#include "GoTask.h"

GoTask::GoTask(Task * parent, double distance, String name) : Task(GO, parent, name) {
  this->distance = distance;
}

GoTask::GoTask(Task * mom, Task * dad, double distance, String name) : Task(GO, mom, dad, name) {
  this->distance = distance;
}

bool GoTask::update() {

  if(distance > 0) {
    leftMotor.set(20, Stepper::FORWARD);
    rightMotor.set(20, Stepper::FORWARD);
  } else {
    leftMotor.set(20, Stepper::BACKWARD);
    rightMotor.set(20, Stepper::BACKWARD);
  }

  if(leftMotor.getCount() > steps) {
    leftMotor.stop();
    rightMotor.stop();
    return true;
  }
  
  return false;

}

void GoTask::init() {
  
  Task::init();

  leftMotor.resetCount();
  rightMotor.resetCount();
  
  Serial.print("Distance to travel: ");
  Serial.print(distance);
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
  
  steps = distance / distPerStep;
  
  Serial.print("Need to step: ");
  Serial.print(steps);
  Serial.print(" times.\n");
  
}
