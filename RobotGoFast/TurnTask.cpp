#include "TurnTask.h"

TurnTask::TurnTask(Task * parent, float deg, String name) : Task(TURN, parent, name), pid(0.05, 0.0, 0.15, -12.0, 12.0) {
  this->deg = deg;
}

TurnTask::TurnTask(Task * mom, Task * dad, float deg, String name) : Task(TURN, mom, dad, name), pid(0.05, 0.0, 0.15, -12.0, 12.0) {
  this->deg = deg;
}

bool TurnTask::update() {

  // Start per gyro code

  /*Vec2 currentVec = Vec2::fromPolarDeg(1.0, imu.yaw);

  float angle = currentVec.angleTo(finalVec);
  angle *= 180.f / PI;

  double speed;

  if(angle > 0) {

    speed = pid.Compute(angle, 0);

    // Turn right
    Serial.println(" Right");
    leftMotor.set(abs(speed), Stepper::FORWARD);
    rightMotor.set(abs(speed), Stepper::BACKWARD);

  } else {

    speed = pid.Compute(angle, 0);

    // Turn left
    Serial.println(" Left");
    leftMotor.set(abs(speed), Stepper::BACKWARD);
    rightMotor.set(abs(speed), Stepper::FORWARD);

  }

  Serial.print("Ang ");
  printDouble(angle, 100);
  Serial.print(" Spd ");
  printDouble(speed, 100);
  //Serial.println();

  if(abs(angle) < 1.0) {
    samples--;
  }

  if(samples <= 0) {
    leftMotor.stop();
    rightMotor.stop();
    return true;
  }
  
  return false;*/

  // Start per steps code

  double speed;

  if(direction) {
    steps -= leftMotor.getCount() + rightMotor.getCount() / 2;
  } else {
    steps += leftMotor.getCount() + rightMotor.getCount() / 2;
  }

  if(steps > 0) {

    // Turn right
    speed = pid.Compute(steps, 0);

    Serial.println("Right ");
    leftMotor.set(abs(speed), Stepper::FORWARD);
    rightMotor.set(abs(speed), Stepper::BACKWARD);

    direction = true;

  } else {

    speed = pid.Compute(steps, 0);

    // Turn left
    Serial.println("Left ");
    leftMotor.set(abs(speed), Stepper::BACKWARD);
    rightMotor.set(abs(speed), Stepper::FORWARD);

    direction = false;

  }

  Serial.print("Steps ");
  printDouble(steps, 100);
  Serial.print(" Spd ");
  printDouble(speed, 100);
  Serial.println();

  if(abs(steps) <= 5) {
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

  // Capture IMU yaw
  initialVec = Vec2::fromPolarDeg(1.0, imu.yaw);
  finalVec = Vec2::fromPolarDeg(1.0, fmod(imu.yaw + deg, 360));

  // Calculate distance to move
  double distnce = PI * Stepper::wheelSpacing * deg / 360.f;

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

