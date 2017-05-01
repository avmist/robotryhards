#include "GoTask2.h"

GoTask2::GoTask2(Task * parent, float distance, String name) : Task(GO, parent, name) {
  this->distance = distance;
}

GoTask2::GoTask2(Task * mom, Task * dad, float distance, String name) : Task(GO, mom, dad, name) {
  this->distance = distance;
}

void GoTask2::initPID() {
  distPD = PID(0.05, 0.0, 0.15, -6.0, 6.0);
  angPD = PID(0.2, 0.0, 0.1, -6.0, 6.0);
}

bool GoTask2::update() {

  float d0 = ir0.read();
  /*if(d0 == LinearFit::TOO_FAR) {
    SerialUSB.print(" FAR ");
  } else if(d0 == LinearFit::TOO_CLOSE) {
    SerialUSB.print("CLOSE");
  } else {
    printDouble(d0, 100);
  }*/
  
  //SerialUSB.print(" ");
  
  float d1 = ir1.read();
  /*if(d1 == LinearFit::TOO_FAR) {
    SerialUSB.print(" FAR ");
  } else if(d1 == LinearFit::TOO_CLOSE) {
    SerialUSB.print("CLOSE");
  } else {
    printDouble(d1, 100);
  }*/
  
  //SerialUSB.print(" ");

  float aveDistLeft = (d0 + d1) / 2.0;
  
  float d2 = ir2.read();
  /*if(d2 == LinearFit::TOO_FAR) {
    SerialUSB.print(" FAR ");
  } else if(d2 == LinearFit::TOO_CLOSE) {
    SerialUSB.print("CLOSE");
  } else {
    printDouble(d2, 100);
  }*/
  
  //SerialUSB.print(" ");
  
  float d3 = ir3.read();
  /*if(d3 == LinearFit::TOO_FAR) {
    SerialUSB.print(" FAR ");
  } else if(d3 == LinearFit::TOO_CLOSE) {
    SerialUSB.print("CLOSE");
  } else {
    printDouble(d3, 100);
  }*/
  
  //SerialUSB.print(" ");

  float aveDistRight = (d2 + d3) / 2.0;

  float distError, angError;

  if(d0 != LinearFit::TOO_FAR && d1 != LinearFit::TOO_FAR && d2 != LinearFit::TOO_FAR && d3 != LinearFit::TOO_FAR) {

    // Both sides against wall - best case for wall tracking

    float distError = aveDistRight - aveDistLeft;

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

    float diff = angPD.Compute(angleLeft, targetAngle);

    SerialUSB.print("Diff: ");
    printDouble(diff, 10);
    SerialUSB.println();

    leftMotor.set(6 + diff, Stepper::FORWARD);
    rightMotor.set(6 - diff, Stepper::FORWARD);

    printDouble(6 + diff, 100);
    SerialUSB.print(" jrlkwjr ");
    printDouble(6 - diff, 100);
    SerialUSB.println();

    // leftMotor.set(0, Stepper::FORWARD);
    // rightMotor.set(0, Stepper::FORWARD);

    SerialUSB.println();

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

    // Serial update
    if(state != BOTH_WALLS) {
      SerialUSB.println("BOTH");
      state = BOTH_WALLS;
    }

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
      //SerialUSB.println("LEFT");
      state = LEFT_WALL;
    }
    
    // LED blink
    if((micros() - lastStatusPing) > 500000) {
      led.solid(LED::BLUE);
      led.off(LED::RED);
      lastStatusPing = micros();
    }
    
  } else if(d2 != LinearFit::TOO_FAR && d3 != LinearFit::TOO_FAR) {

    leftMotor.set(0, Stepper::FORWARD);
    rightMotor.set(0, Stepper::FORWARD);

    // Right side against wall

    // Serial update
    if(state != RIGHT_WALL) {
      //SerialUSB.println("RIGHT");
      state = RIGHT_WALL;
    }

    // LED blink
    if((micros() - lastStatusPing) > 500000) {
      led.solid(LED::RED);
      led.off(LED::BLUE);
      lastStatusPing = micros();
    }
    
  } else {

    // Serial update
    if(state != NO_WALL) {
      //SerialUSB.println("NO WALL");
      state = NO_WALL;
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
  
  steps = distance / distPerStep;
  
  SerialUSB.print("Need to step: ");
  SerialUSB.print(steps);
  SerialUSB.print(" times.\n");

  lastStatusPing = micros();

  initPID();
  
}
