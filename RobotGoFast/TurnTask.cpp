#include "TurnTask.h"

TurnTask::TurnTask() {
  
}

TurnTask::TurnTask(Task * parent, float deg, String name) : Task(TURN, parent, name) {
  this->deg = deg;
}

int TurnTask::update() {

  //SerialUSB.println("Turn update...");

  // Start per steps code

  float speed = pid.Compute(steps, 0);

  if(steps > 0) {

    // Turn right
    
    //SerialUSB.println("Right ");

    noInterrupts();
    leftMotor.set(abs(speed), Stepper::FORWARD);
    rightMotor.set(abs(speed), Stepper::BACKWARD);
    interrupts();

    direction = true;

  } else {

    // Turn left
    //SerialUSB.println("Left ");

    noInterrupts();
    leftMotor.set(abs(speed), Stepper::BACKWARD);
    rightMotor.set(abs(speed), Stepper::FORWARD);
    interrupts();

    direction = false;

  }

  if(direction) {
    steps -= leftMotor.getCount() + rightMotor.getCount() / 2;
  } else {
    steps += leftMotor.getCount() + rightMotor.getCount() / 2;
  }

  // SerialUSB.print("Steps ");
  // printDouble(steps, 100);
  // SerialUSB.print(" Spd ");
  // printDouble(speed, 100);
  // SerialUSB.println();

  if(abs(steps) <= 5) {

    noInterrupts();
    leftMotor.stop();
    rightMotor.stop();
    interrupts();

    traversed = true;
    return END;

  }

  return CONTINUE;

}

void TurnTask::init() {
  
  Task::init();

  pid = PID(0.05, 0.0, 0.15, -12.0, 12.0);

  leftMotor.resetCount();
  rightMotor.resetCount();

  // Calculate distance to move
  float distnce = PI * Stepper::wheelSpacing * backtracking * deg / 360.f;

  SerialUSB.print("Arc distance to travel: ");
  SerialUSB.print(distnce);
  SerialUSB.print(" in.\n");

  // Calculate degree per step
  float degPerStep = 360.f / (Stepper::stepsPerRevolution * Stepper::microsteps);

  SerialUSB.print("Deg per step: ");
  SerialUSB.print(degPerStep);
  SerialUSB.print(" deg.\n");

  // Calculate distance per step
  float distPerStep = PI * Stepper::wheelDiameter * (degPerStep / 360.f);

  SerialUSB.print("Dist per step: ");
  SerialUSB.print(distPerStep);
  SerialUSB.print(" in.\n");
  
  steps = distnce / distPerStep;
  steps *= 1.1;
  
  SerialUSB.print("Need to step: ");
  SerialUSB.print(steps);
  SerialUSB.print(" times.\n");
  
}

bool TurnTask::between(float value, float low, float high) {

  if(low > high) {
    return (value >= high && value <= low);
  } else {
    return (value >= low && value <= high);
  }

}

