#include "GoTask2.h"

GoTask2::GoTask2() : distance(0) {

}

GoTask2::GoTask2(Task * parent, float distance, String name) : Task(GO2, parent, name), distance(distance) {
  dontTrack = false;
}

GoTask2::GoTask2(Task * parent, float distance, String name, bool dontTrack) : Task(GO2, parent, name), distance(distance) {
  this->dontTrack = dontTrack;
}

void GoTask2::initPID() {
  distPD = PID(0.8, 0.0, 0.4, -6.0, 6.0);
  angPD = PID(0.4, 0.0, 0.2, -4.0, 4.0);
}

int GoTask2::update() {

  // ToF code from adafruit exmaple
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    //SerialUSB.print("Range: "); SerialUSB.println(range);
  }

  // Some error occurred, print it out!
  
  if((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    SerialUSB.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    //SerialUSB.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    //SerialUSB.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    SerialUSB.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    //SerialUSB.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    //SerialUSB.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    //SerialUSB.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    //SerialUSB.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    //SerialUSB.println("Range reading overflow");
  }

  // Convert mm to in
  float distnace = range * 0.0393701;

  if(distnace < 1.5 && backtracking == NOT_BACKTRACKING) {

    noInterrupts();
    leftMotor.stop();
    rightMotor.stop();
    interrupts();

    float degPerStep = 360.f / (Stepper::stepsPerRevolution * Stepper::microsteps);
    float distPerStep = PI * Stepper::wheelDiameter * (degPerStep / 360.f);
    int closeEnoughStepCount = 3.0 / distPerStep;

    // Are we close enough to our target?

    if(abs(stepsToTravel - stepsTraveled) < closeEnoughStepCount) {

      SerialUSB.println("Close enough complete.");

      traversed = true;
      return END;

    } else {

      SerialUSB.println();
      SerialUSB.println("Obstacle detected.");
      SerialUSB.println();

      float degPerStep = 360.f / (Stepper::stepsPerRevolution * Stepper::microsteps);
      float distPerStep = PI * Stepper::wheelDiameter * (degPerStep / 360.f);

      SerialUSB.println((int) stepsTraveled);
      // Add one inch to account for stopping drift
      stepsTraveled += 1.25 / distPerStep;
      SerialUSB.println((int) stepsTraveled);
      SerialUSB.println();

      backtracking = BACKTRACKING;
      stepsToTravel = stepsTraveled;
      stepsTraveled = 0;

    }

  }

  float d0 = ir0.read();  
  float d1 = ir1.read();
  float aveDistLeft = (d0 + d1) / 2.0;
  
  float d2 = ir2.read();
  float d3 = ir3.read();
  float aveDistRight = (d2 + d3) / 2.0;

  float angleLeft = backtracking * atan2(d0 - d1, IR_SPACING);
  angleLeft = angleLeft * 180.0 / PI;

  float angleRight = backtracking * atan2(d2 - d3, IR_SPACING);
  angleRight = angleRight * 180.0 / PI;

  if(d0 < 6.5 && d1 < 6.5 && d2 < 6.5 && d3 < 6.5 && !dontTrack) {

    // Both sides against wall - best case for wall tracking

    // SerialUSB update
    if(wallState != BOTH_WALLS) {

      SerialUSB.println("BOTH");
      wallState = BOTH_WALLS;
    }

    float overallAngle = angleRight + angleLeft;
    float targetAngle = overallAngle / 2.0;
    float angError = targetAngle - angleLeft;

    // SerialUSB.print("Ang Error: ");
    // printDouble(angError, 10);
    // SerialUSB.println();

    float distError = aveDistLeft - aveDistRight;

    float distDiff = distPD.Compute(distError, 0.0);
    float angDiff = angPD.Compute(angleLeft, targetAngle);

    // SerialUSB.print("Ang diff: ");
    // printDouble(angDiff, 10);
    // SerialUSB.println();

    // SerialUSB.print("Dist diff: ");
    // printDouble(distDiff, 10);
    // SerialUSB.println();

    float degPerStep = 360.f / (Stepper::stepsPerRevolution * Stepper::microsteps);
    float distPerStep = PI * Stepper::wheelDiameter * (degPerStep / 360.f);

    // Add one inch to account for stopping drift
    unsigned long driftDist = 13 / distPerStep;

    if(stepsToTravel < driftDist) {
      distDiff = distDiff / 2;
    }

    if(backtracking == NOT_BACKTRACKING) {

      noInterrupts();
      leftMotor.set(8 + (angDiff + distDiff), Stepper::FORWARD);
      rightMotor.set(8 - (angDiff + distDiff), Stepper::FORWARD);
      interrupts();

    } else {

      noInterrupts();
      leftMotor.set(8 + (angDiff + distDiff), Stepper::BACKWARD);
      rightMotor.set(8 - (angDiff + distDiff), Stepper::BACKWARD);
      interrupts();

    }

    // printDouble(8 + (angDiff + distDiff), 100);
    // SerialUSB.print(" ");
    // printDouble(8 - (angDiff + distDiff), 100);
    // SerialUSB.println();

    // leftMotor.set(0, Stepper::FORWARD);
    // rightMotor.set(0, Stepper::FORWARD);

    // SerialUSB.println();

    // LED blink
    if((micros() - lastStatusPing) > 500000) {
      led.solid(LED::BLUE);
      led.solid(LED::RED);
      lastStatusPing = micros();
    }
    
  } else if(d0 < 6.5 && d1 < 6.5 && !dontTrack) {

    // Left side against wall

    // SerialUSB update
    if(wallState != LEFT_WALL) {
      SerialUSB.println("LEFT");
      wallState = LEFT_WALL;
    }

    float distDiff = distPD.Compute(aveDistLeft, 5.5);
    float angDiff = angPD.Compute(-angleLeft, 0.0);

    float degPerStep = 360.f / (Stepper::stepsPerRevolution * Stepper::microsteps);
    float distPerStep = PI * Stepper::wheelDiameter * (degPerStep / 360.f);

    unsigned long driftDist = 13 / distPerStep;

    if(stepsToTravel < driftDist) {
      distDiff = 0;
    }

    if(backtracking == NOT_BACKTRACKING) {
      noInterrupts();
      leftMotor.set(8 + (angDiff + distDiff), Stepper::FORWARD);
      rightMotor.set(8 - (angDiff + distDiff), Stepper::FORWARD);
      interrupts();
    } else {
      noInterrupts();
      leftMotor.set(8 + (angDiff + distDiff), Stepper::BACKWARD);
      rightMotor.set(8 - (angDiff + distDiff), Stepper::BACKWARD);
      interrupts();
    }

    // printDouble(8 + (angDiff + distDiff), 100);
    // SerialUSB.print(" ");
    // printDouble(8 - (angDiff + distDiff), 100);
    // SerialUSB.println();
    
    // LED blink
    if((micros() - lastStatusPing) > 500000) {
      led.solid(LED::BLUE);
      led.off(LED::RED);
      lastStatusPing = micros();
    }
    
  } else if(d2 < 6.5 && d3 < 6.5 && !dontTrack) {

    // Right side against wall

    // SerialUSB update
    if(wallState != RIGHT_WALL) {
      SerialUSB.println("RIGHT");
      wallState = RIGHT_WALL;
    }

    float distDiff = distPD.Compute(aveDistRight, 5.5);
    float angDiff = angPD.Compute(-angleRight, 0.0);

    float degPerStep = 360.f / (Stepper::stepsPerRevolution * Stepper::microsteps);
    float distPerStep = PI * Stepper::wheelDiameter * (degPerStep / 360.f);

    unsigned long driftDist = 13 / distPerStep;

    if(stepsToTravel < driftDist) {
      distDiff = 0;
    }

    if(backtracking == NOT_BACKTRACKING) {

      noInterrupts();
      leftMotor.set(8 + (angDiff + distDiff), Stepper::FORWARD);
      rightMotor.set(8 - (angDiff + distDiff), Stepper::FORWARD);
      interrupts();

    } else {

      noInterrupts();
      leftMotor.set(8 + (angDiff + distDiff), Stepper::BACKWARD);
      rightMotor.set(8 - (angDiff + distDiff), Stepper::BACKWARD);
      interrupts();

    }

    // printDouble(8 + (angDiff + distDiff), 100);
    // SerialUSB.print(" ");
    // printDouble(8 - (angDiff + distDiff), 100);
    // SerialUSB.println();

    // LED blink
    if((micros() - lastStatusPing) > 500000) {
      led.solid(LED::RED);
      led.off(LED::BLUE);
      lastStatusPing = micros();
    }
    
  } else {

    // SerialUSB update
    if(wallState != NO_WALL) {

      SerialUSB.println("NO WALL");
      wallState = NO_WALL;

      // Go 8 in
      // float degPerStep = 360.f / (Stepper::stepsPerRevolution * Stepper::microsteps);
      // float distPerStep = PI * Stepper::wheelDiameter * (degPerStep / 360.f);

      // unsigned long fixedDist = 6.0 / distPerStep;

      // if(stepsToTravel > fixedDist * 0.90 && stepsToTravel < fixedDist * 1.10) {

      //   SerialUSB.println("Junction near end detected, going fixed distance.");
      //   stepsToTravel = fixedDist;

      //   led.solid(LED::RED);
      //   led.solid(LED::GREEN);
      //   led.solid(LED::BLUE);

      // }

    }

    if(backtracking == NOT_BACKTRACKING) {

      noInterrupts();
      leftMotor.set(7, Stepper::FORWARD);
      rightMotor.set(7, Stepper::FORWARD);
      interrupts();

    } else {

      noInterrupts();
      leftMotor.set(7, Stepper::BACKWARD);
      rightMotor.set(7, Stepper::BACKWARD);
      interrupts();

    }

  }

  // Get count
  stepsTraveled += (leftMotor.getCount() + rightMotor.getCount()) / 2.0;
  //SerialUSB.println(stepsTraveled);
  
  if(stepsTraveled > stepsToTravel) {

    noInterrupts();
    leftMotor.stop();
    rightMotor.stop();
    interrupts();

    traversed = true;

    if(backtracking == BACKTRACKING) {

      SerialUSB.println("Backtracking complete (went to distance).");

      return END;

    } else {

      SerialUSB.println("Normal complete (went to distance).");

      return END;

    }

  }
  
  return CONTINUE;

}

void GoTask2::init() {

  Task::init();

  wallState = NO_WALL;

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