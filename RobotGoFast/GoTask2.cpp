#include "GoTask2.h"

GoTask2::GoTask2(Task * parent, float distance, String name) : Task(GO, parent, name) {
  this->distance = distance;
}

GoTask2::GoTask2(Task * mom, Task * dad, float distance, String name) : Task(GO, mom, dad, name) {
  this->distance = distance;
}

void GoTask2::initPID() {
  distPD = PID(0.8, 0.0, 0.4, -6.0, 6.0);
  angPD = PID(0.4, 0.0, 0.2, -4.0, 4.0);
}

bool GoTask2::update() {

  float d0 = ir0.read();
  float d1 = ir1.read();
  float d2 = ir2.read();
  float d3 = ir3.read();
  float aveDistLeft = (d0 + d1) / 2.0;
  float aveDistRight = (d2 + d3) / 2.0;

  if(d0 != LinearFit::TOO_FAR && d1 != LinearFit::TOO_FAR && d2 != LinearFit::TOO_FAR && d3 != LinearFit::TOO_FAR) {

    // Both sides against wall - best case for wall tracking

    // SerialUSB update
    if(state != BOTH_WALLS) {
      SerialUSB.println("BOTH");
      state = BOTH_WALLS;
    }

    float angleLeft = atan2(d0 - d1, IR_SPACING);
    angleLeft = angleLeft * 180.0 / PI;

    float angleRight = atan2(d2 - d3, IR_SPACING);
    angleRight = angleRight * 180.0 / PI;

    float overallAngle = angleRight + angleLeft;
    float targetAngle = overallAngle / 2.0;
    float angError = targetAngle - angleLeft;

    SerialUSB.print("Ang Error: ");
    printDouble(angError, 10);
    SerialUSB.println();

    float distError = aveDistLeft - aveDistRight;

    float distDiff = distPD.Compute(distError, 0.0);
    float angDiff = angPD.Compute(angleLeft, targetAngle);

    SerialUSB.print("Ang diff: ");
    printDouble(angDiff, 10);
    SerialUSB.println();

    SerialUSB.print("Dist diff: ");
    printDouble(distDiff, 10);
    SerialUSB.println();

    //leftMotor.set(8 + (angDiff + distDiff), Stepper::FORWARD);
    //rightMotor.set(8 - (angDiff + distDiff), Stepper::FORWARD);

    printDouble(8 + (angDiff + distDiff), 100);
    SerialUSB.print(" ");
    printDouble(8 - (angDiff + distDiff), 100);
    SerialUSB.println();

    // leftMotor.set(0, Stepper::FORWARD);
    // rightMotor.set(0, Stepper::FORWARD);

    SerialUSB.println();

    // LED blink
    if((micros() - lastStatusPing) > 500000) {
      led.solid(LED::BLUE);
      led.solid(LED::RED);
      lastStatusPing = micros();
    }
    
  } else if(d0 != LinearFit::TOO_FAR && d1 != LinearFit::TOO_FAR) {

    leftMotor.set(0, Stepper::FORWARD);
    rightMotor.set(0, Stepper::FORWARD);

    // Left side against wall

    // SerialUSB update
    if(state != LEFT_WALL) {
      SerialUSB.println("LEFT");
      state = LEFT_WALL;
    }
    
    // LED blink
    if((micros() - lastStatusPing) > 500000) {
      led.solid(LED::BLUE);
      led.off(LED::RED);
      lastStatusPing = micros();
    }
    
  } else if(d2 != LinearFit::TOO_FAR && d3 != LinearFit::TOO_FAR) {

    // Right side against wall

    // SerialUSB update
    if(state != RIGHT_WALL) {
      SerialUSB.println("RIGHT");
      state = RIGHT_WALL;
    }

    leftMotor.set(0, Stepper::FORWARD);
    rightMotor.set(0, Stepper::FORWARD);

    // LED blink
    if((micros() - lastStatusPing) > 500000) {
      led.solid(LED::RED);
      led.off(LED::BLUE);
      lastStatusPing = micros();
    }
    
  } else {

    // SerialUSB update
    if(state != NO_WALL) {
      SerialUSB.println("NO WALL");
      state = NO_WALL;
    }

    leftMotor.set(0, Stepper::FORWARD);
    rightMotor.set(0, Stepper::FORWARD);

  }

  // Get count
  stepsTraveled += (leftMotor.getCount() + rightMotor.getCount()) / 2.0;
  //SerialUSB.println(stepsTraveled);
  
  if(stepsTraveled > stepsToTravel) {
    leftMotor.stop();
    rightMotor.stop();
    return true;
  }
  
  return false;

}

void GoTask2::init() {

  state = NO_WALL;
  
  Task::init();

  leftMotor.resetCount();
  rightMotor.resetCount();
  
  SerialUSB.print("Distance to travel: ");
  SerialUSB.print(distance);
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
  
  stepsToTravel = distance / distPerStep;
  stepsTraveled = 0;
  
  SerialUSB.print("Need to step: ");
  SerialUSB.print(stepsToTravel);
  SerialUSB.print(" times.\n");

  lastStatusPing = micros();

  initPID();
  
}