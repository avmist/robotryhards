#include "GoTask.h"

GoTask::GoTask(Task * parent, double distance, String name) : Task(GO, parent, name) {
  this->distance = distance;
}

GoTask::GoTask(Task * mom, Task * dad, double distance, String name) : Task(GO, mom, dad, name) {
  this->distance = distance;
}

bool GoTask::update() {

  double d0 = ir0.read();
  if(d0 == LinearFit::TOO_FAR) {
    Serial.print(" FAR ");
  } else if(d0 == LinearFit::TOO_CLOSE) {
    Serial.print("CLOSE");
  } else {
    printDouble(d0, 100);
  }
  
  Serial.print(" ");
  
  if(d1 == LinearFit::TOO_FAR) {
    Serial.print(" FAR ");
  } else if(d1 == LinearFit::TOO_CLOSE) {
    Serial.print("CLOSE");
  } else {
    printDouble(d1, 100);
  }
  
  Serial.print(" ");

  double aveDistLeft = (d0 + d1) / 2.0;
  
  double d2 = ir2.read();
  if(d2 == LinearFit::TOO_FAR) {
    Serial.print(" FAR ");
  } else if(d2 == LinearFit::TOO_CLOSE) {
    Serial.print("CLOSE");
  } else {
    printDouble(d2, 100);
  }
  
  Serial.print(" ");
  
  double d3 = ir3.read();
  if(d3 == LinearFit::TOO_FAR) {
    Serial.print(" FAR ");
  } else if(d3 == LinearFit::TOO_CLOSE) {
    Serial.print("CLOSE");
  } else {
    printDouble(d3, 100);
  }
  
  Serial.print(" ");

  double aveDistRight = (d2 + d3) / 2.0;

  // Calculate angle on left
  double angleLeft = atan(abs(d0 - d1) / IR_SPACING);
  angleLeft = angleLeft * 180.0 / PI;
  
  if(d0 > d1) {
    angleLeft *= -1;
  }
  
  Serial.print(" l-a ");
  printDouble(angleLeft, 100);
  
  Serial.print(" ");

  // Calculate angle on right
  double angleRight = atan(abs(d2 - d3) / IR_SPACING);
  angleRight = angleRight * 180.0 / PI;
  
  if(d2 > d3) {
    angleRight *= -1;
  }
  
  Serial.print(" r-a ");
  printDouble(angleRight, 100);
  Serial.println();

  if(d0 != LinearFit::TOO_FAR && d1 != LinearFit::TOO_FAR && d2 != LinearFit::TOO_FAR && d3 != LinearFit::TOO_FAR) {

    // Both sides against wall - best case for wall tracking

    if((micros() - lastStatusPing) > 500000) {
      led.blink(LED::BLUE, 250000);
      led.blink(LED::RED, 250000);
      lastStatusPing = micros();
      Serial.println("Both walls detected");
    }
    
  } else if(d0 != LinearFit::TOO_FAR && d1 != LinearFit::TOO_FAR) {

    // Left side against wall
    
    if((micros() - lastStatusPing) > 500000) {
      led.blink(LED::BLUE, 250000);
      lastStatusPing = micros();
      Serial.println("Left wall detected");
    }
    
  } else if(d2 != LinearFit::TOO_FAR && d3 != LinearFit::TOO_FAR) {

    // Right side against wall

    if((micros() - lastStatusPing) > 500000) {
      led.blink(LED::RED, 250000);
      lastStatusPing = micros();
      Serial.println("Right wall detected");
    }
    
  }

  /*if(distance > 0) {
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
  }*/
  
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

  lastStatusPing = micros();
  
}
