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
  /*if(d0 == LinearFit::TOO_FAR) {
    Serial.print(" FAR ");
  } else if(d0 == LinearFit::TOO_CLOSE) {
    Serial.print("CLOSE");
  } else {
    printDouble(d0, 100);
  }*/
  
  //Serial.print(" ");
  
  float d1 = ir1.read();
  /*if(d1 == LinearFit::TOO_FAR) {
    Serial.print(" FAR ");
  } else if(d1 == LinearFit::TOO_CLOSE) {
    Serial.print("CLOSE");
  } else {
    printDouble(d1, 100);
  }*/
  
  //Serial.print(" ");

  float aveDistLeft = (d0 + d1) / 2.0;
  
  float d2 = ir2.read();
  /*if(d2 == LinearFit::TOO_FAR) {
    Serial.print(" FAR ");
  } else if(d2 == LinearFit::TOO_CLOSE) {
    Serial.print("CLOSE");
  } else {
    printDouble(d2, 100);
  }*/
  
  //Serial.print(" ");
  
  float d3 = ir3.read();
  /*if(d3 == LinearFit::TOO_FAR) {
    Serial.print(" FAR ");
  } else if(d3 == LinearFit::TOO_CLOSE) {
    Serial.print("CLOSE");
  } else {
    printDouble(d3, 100);
  }*/
  
  //Serial.print(" ");

  float aveDistRight = (d2 + d3) / 2.0;

  if(d0 != LinearFit::TOO_FAR && d1 != LinearFit::TOO_FAR && d2 != LinearFit::TOO_FAR && d3 != LinearFit::TOO_FAR) {

    // Both sides against wall - best case for wall tracking

    // Serial update
    if(state != BOTH_WALLS) {
      Serial.println("BOTH");
      state = BOTH_WALLS;
    }

    float angleLeft = atan2(d0 - d1, IR_SPACING);
    angleLeft = angleLeft * 180.0 / PI;

    float angleRight = atan2(d2 - d3, IR_SPACING);
    angleRight = angleRight * 180.0 / PI;

    float overallAngle = angleRight + angleLeft;
    float targetAngle = overallAngle / 2.0;
    float angError = targetAngle - angleLeft;

    Serial.print("Ang Error: ");
    printDouble(angError, 10);
    Serial.println();

    float distError = aveDistLeft - aveDistRight;

    float distDiff = distPD.Compute(distError, 0.0);
    float angDiff = angPD.Compute(angleLeft, targetAngle);

    Serial.print("Ang diff: ");
    printDouble(angDiff, 10);
    Serial.println();

    Serial.print("Dist diff: ");
    printDouble(distDiff, 10);
    Serial.println();

    leftMotor.set(8 + (angDiff + distDiff), Stepper::FORWARD);
    rightMotor.set(8 - (angDiff + distDiff), Stepper::FORWARD);

    printDouble(8 + (angDiff + distDiff), 100);
    Serial.print(" ");
    printDouble(8 - (angDiff + distDiff), 100);
    Serial.println();

    // leftMotor.set(0, Stepper::FORWARD);
    // rightMotor.set(0, Stepper::FORWARD);

    Serial.println();

    // if(angleLeft >= 0.0 && angleRight >= 0.0) {

    //   // Best case, both walls are angling outwards relative to us

    //   // Simply match the angles;

    //   float overallAngle = angleRight + angleLeft;
    //   float targetAngle = overallAngle / 2.0;
    //   float angError = targetAngle - angleLeft;

    // } else if(angleLeft >= 0.0 && angleRight <= 0.0) {

    //   // Robot needs to turn left

    //   // Determine the overall angle of the junction
    //   float overallAngle = angleLeft + angleRight;
    //   float targetAngle = overallAngle / 2.0;
    //   float angError = targetAngle - angleLeft;

    // } else if(angleLeft <= 0.0 && angleRight >= 0.0) {

    //   // Robot needs to turn Right

    //   // Determine the overall angle of the junction
    //   float overallAngle = angleLeft + angleRight;
    //   float targetAngle = overallAngle / 2.0;
    //   float angError = angleRight - targetAngle;
      
    // } else {
      
    // }



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

    // Serial update
    if(state != LEFT_WALL) {
      Serial.println("LEFT");
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

    // Serial update
    if(state != RIGHT_WALL) {
      Serial.println("RIGHT");
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

    // Serial update
    if(state != NO_WALL) {
      Serial.println("NO WALL");
      state = NO_WALL;
    }

    leftMotor.set(0, Stepper::FORWARD);
    rightMotor.set(0, Stepper::FORWARD);

  }

  // Get count
  stepsTraveled += (leftMotor.getCount() + rightMotor.getCount()) / 2.0;
  //Serial.println(stepsTraveled);
  
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
  
  Serial.print("Distance to travel: ");
  Serial.print(distance);
  Serial.print(" in.\n");

  // Calculate degree per step
  float degPerStep = 360.f / (Stepper::stepsPerRevolution * Stepper::microsteps);

  Serial.print("Deg per step: ");
  Serial.print(degPerStep);
  Serial.print(" deg.\n");

  // Calculate distance per step
  float distPerStep = PI * Stepper::wheelDiameter * (degPerStep / 360.f);

  Serial.print("Dist per step: ");
  Serial.print(distPerStep);
  Serial.print(" in.\n");
  
  stepsToTravel = distance / distPerStep;
  stepsTraveled = 0;
  
  Serial.print("Need to step: ");
  Serial.print(stepsToTravel);
  Serial.print(" times.\n");

  lastStatusPing = micros();

  initPID();
  
}
