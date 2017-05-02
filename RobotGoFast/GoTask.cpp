#include "GoTask.h"

GoTask::GoTask(Task * parent, float distance, String name) : Task(GO, parent, name) {
  this->distance = distance;
}

GoTask::GoTask(Task * mom, Task * dad, float distance, String name) : Task(GO, mom, dad, name) {
  this->distance = distance;
}

bool GoTask::update() {

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

  //float aveDistRight = (d2 + d3) / 2.0;	//TODO: find out what this does
  //Calculate seen wall points
  Vec2 i0 = Vec2(IR_SPACING / 2.0, d0);
  Vec2 i1 = Vec2(-IR_SPACING / 2.0, d1);
  Vec2 i2 = Vec2(-IR_SPACING / 2.0, -d2);
  Vec2 i3 = Vec2(IR_SPACING / 2.0, -d3);
  
  //Calculate left boid
  Vec2 leftDir = i0 - i1;
  Vec2 leftPos = Vec2::intersect(i1, leftDir, Vec2(0.f, 0.f), leftDir.left());
  
  // Calculate right boid
  Vec2 rightDir = i3 - i2;
  Vec2 rightPos = Vec2::intersect(i2, rightDir, Vec2(0.f, 0.f), rightDir.right());

  //Calculate Intent
  Vec2 intent = (rightPos.unit() * -leftPos.size() + leftPos.unit() * -rightPos.size()).unit();	//avoid
  SerialUSB.print("Avoid: ");
  printDouble(intent.x, 10);
  SerialUSB.print(",   ");
  printDouble(intent.y, 10);
  intent = intent + (leftDir * rightPos.size() + rightDir * leftPos.size()).unit();	//follow
  SerialUSB.print("Follow: ");
  printDouble(intent.unit().x, 10);
  SerialUSB.print(",   ");
  printDouble(intent.unit().y, 10);
  float turnAmt = Vec2(1.f, 0.f).cross(intent.unit());

  leftMotor.set(15 + turnAmt * 5, Stepper::FORWARD);
  rightMotor.set(15 - turnAmt * 5, Stepper::FORWARD);

  SerialUSB.println();

  if(d0 != LinearFit::TOO_FAR && d1 != LinearFit::TOO_FAR && d2 != LinearFit::TOO_FAR && d3 != LinearFit::TOO_FAR) {

    // Both sides against wall - best case for wall tracking

    if((micros() - lastStatusPing) > 500000) {
      led.solid(LED::BLUE);
      led.solid(LED::RED);
      lastStatusPing = micros();
      //SerialUSB.println("Both walls detected");
    }
    
  } else if(d0 != LinearFit::TOO_FAR && d1 != LinearFit::TOO_FAR) {

    // Left side against wall
    
    if((micros() - lastStatusPing) > 500000) {
      led.solid(LED::BLUE);
      led.off(LED::RED);
      lastStatusPing = micros();
      //SerialUSB.println("Left wall detected");
    }
    
  } else if(d2 != LinearFit::TOO_FAR && d3 != LinearFit::TOO_FAR) {

    // Right side against wall

    if((micros() - lastStatusPing) > 500000) {
      led.solid(LED::RED);
      led.off(LED::BLUE);
      lastStatusPing = micros();
      //SerialUSB.println("Right wall detected");
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
  
}
