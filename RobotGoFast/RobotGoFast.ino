//#include <AnalogSmooth.h>
#include "Stepper.h"
#include "Task.h"
#include "RobotState.h"
#include "LED.h"
#include "LinearFit.h"
#include "TurnTask.h"
#include "DelayTask.h"
#include "StartTask.h"
#include "StopTask.h"
#include "GoTask.h"
#include "HLTM.h"
#include "math.h"
#include "IMU.h"

/* ST    P2    X  X  X  X  X  X
 *    X  X     X  X  X  X  X  X
 *    X  X  J2    P5       X  X
 *    X  X  P3 X  X  X     X  X
 * P1       J1    P4       X  X
 * X  X  X  X  X  X  X  J3        
 * X  X  X  X  X  X  X  X  X   
 * X  X  X  X  X  X  X  X  X  P6
 * X  X  X  X  X  X  X  X  X   
 * X  X  X  X  X  X  X  X  X  ED
 */
 
// Chain 1 Start @ Start
/*Task root(NULL, Task::START);
Task t1_1(&root, Task::FORWARD);
Task t1_2(&t1_1, Task::FORWARD);
Task t1_3(&t1_2, Task::FORWARD);
Task t1_4(&t1_3, Task::FORWARD);
TurnTask t1_5(&t1_4, -90);
Task t1_6(&t1_5, Task::FORWARD);
Task t1_7(&t1_6, Task::FORWARD);
Task t1_8(&t1_7, Task::FORWARD);
// Chain 1 End @ J1

// Chain 2 Start @ Start
TurnTask t2_1(&root, -90;
Task t2_2(&t2_1, Task::FORWARD);
Task t2_3(&t2_2, Task::FORWARD);
Task t2_4(&t2_3, Task::FORWARD);
Task t2_5(&t2_4, Task::RIGHT);
Task t2_6(&t2_5, Task::FORWARD);
Task t2_7(&t2_6, Task::FORWARD);
Task t2_8(&t2_7, -90);
// Chain 2 End @ J2

// Chain 3a Start @ J1
Task t3a_1(&t1_8, -90);
Task t3a_2(&t3a_1, Task::FORWARD);
Task t3a_3(&t3a_2, Task::FORWARD);
Task t3a_4(&t3a_4, 90);
// Chain 3a End @ J2

// Chain 3b Start @ J2
Task t3b_1(&t2_8, 90);
Task t3b_2(&t3b_1, Task::FORWARD);
Task t3b_3(&t3b_2, Task::FORWARD);
Task t3b_4(&t3b_3, -90);
// Chain 3b End @ J1

// Chain 4 Start @ J1
Task t4_1(&t1_8, Task::FORWARD);
Task t4_2(&t4_1, Task::FORWARD);
Task t4_3(&t4_2, Task::FORWARD);
Task t4_4(&t4_3, Task::FORWARD);
Task t4_5(&t4_4, 90);
Task t4_6(&t4_5, Task::FORWARD);
// Chain 4 End @ J3

// Chain 5 Start @ J2
Task t5_1(&t2_8, Task::FORWARD);
Task t5_2(&t5_1, Task::FORWARD);
Task t5_3(&t5_2, Task::FORWARD);
Task t5_4(&t5_3, Task::FORWARD);
Task t5_5(&t5_4, 90);
Task t5_6(&t5_5, Task::FORWARD);
Task t5_7(&t5_6, Task::FORWARD);
Task t5_8(&t5_7, Task::FORWARD);
// Chain 5 End @ J3

// Chain 6 Start @ J3
Task t6_1(&t5_8, Task::LEFT);
Task t6_2(&t6_1, Task::FORWARD);
Task t6_3(&t6_2, Task::FORWARD);
Task t6_4(&t6_3, Task::RIGHT);
Task t6_5(&t6_4, Task::FORWARD);
Task t6_6(&t6_5, Task::FORWARD);
Task t6_7(&t6_6, Task::FORWARD);
Task t6_8(&t6_7, Task::FORWARD);
Task t6_9(&t6_8, Task::STOP);*/
// Chain 5 End @ End

// Globals
LinearFit ir0(A0);
LinearFit ir1(A1);
LinearFit ir2(A2);
LinearFit ir3(A3);

// Turn test
StartTask turn(NULL, "Start");
TurnTask turn1(&turn, 90, "Turn 1");
DelayTask turn2(&turn1, 2000000, "Delay 2s");
TurnTask turn3(&turn2, -90, "Turn 2");
DelayTask turn4(&turn3, 2000000, "Delay 2s");
TurnTask turn5(&turn4, 180, "Turn 3");
DelayTask turn6(&turn5, 2000000, "Delay 2s");
TurnTask turn7(&turn6, -180, "Turn 4");
DelayTask turn8(&turn7, 2000000, "Delay 2s");
TurnTask turn9(&turn8, 360, "Turn 5");
DelayTask turn10(&turn9, 2000000, "Delay 2s");
TurnTask turn11(&turn10, -360, "Turn 6");
DelayTask turn12(&turn11, 2000000, "Delay 2s");
StopTask turn13(&turn12, "End");

// Square test
StartTask square(NULL, "Start");
TurnTask square1(&square, 90, "Turn 1");
DelayTask square2(&square1, 2000000, "Delay 2s");
GoTask square3(&square2, 24, "Go 24 in");
TurnTask square4(&square3, 90, "Turn 2");
DelayTask square5(&square4, 2000000, "Delay 2s");
GoTask square6(&square5, 24, "Go 24 in");
TurnTask square7(&square6, 90, "Turn 3");
DelayTask square8(&square7, 2000000, "Delay 2s");
GoTask square9(&square8, 24, "Go 24 in");
TurnTask square10(&square9, 90, "Turn 4");
DelayTask square11(&square10, 2000000, "Delay 2s");
GoTask square12(&square11, 24, "Go 24 in");
StopTask square13(&square12, "End");

/*AnalogSmooth as0 = AnalogSmooth();
AnalogSmooth as1 = AnalogSmooth();
AnalogSmooth as2 = AnalogSmooth();
AnalogSmooth as3 = AnalogSmooth();*/

enum RobotState state = IDLE;

Stepper leftMotor(1, 0, false);
Stepper rightMotor(3, 2, true);

LED led(13, 12, 11);

IMU imu;

HLTM hal(&square);

void setup() {

  Serial.begin(115200);

  analogWriteResolution(10);

  pinMode(Stepper::enablePin, OUTPUT);

  // Disable steppers initially
  digitalWrite(Stepper::enablePin, HIGH);
  
  //ir0.addDatapoint(19, 12);
  //ir0.addDatapoint(22, 11.5);
  //ir0.addDatapoint(27, 11);
  //ir0.addDatapoint(33, 10.5);
  //ir0.addDatapoint(45, 10);
  //ir0.addDatapoint(52, 9.5);
  ir0.addDatapoint(63, 9);
  ir0.addDatapoint(76, 8.5);
  ir0.addDatapoint(87, 8);
  ir0.addDatapoint(101, 7.5);
  ir0.addDatapoint(112, 7);
  ir0.addDatapoint(130, 6.5);
  ir0.addDatapoint(146, 6);
  ir0.addDatapoint(164, 5.5);
  ir0.addDatapoint(188, 5);
  ir0.addDatapoint(220, 4.5);
  ir0.addDatapoint(260, 4);
  ir0.addDatapoint(302, 3.5);
  ir0.addDatapoint(365, 3);
  ir0.addDatapoint(480, 2.5);

  //ir1.addDatapoint(16, 12);
  //ir1.addDatapoint(22, 11.5);
  //ir1.addDatapoint(25, 11);
  //ir1.addDatapoint(33, 10.5);
  //ir1.addDatapoint(39, 10);
  //ir1.addDatapoint(51, 9.5);
  ir1.addDatapoint(69, 9);
  ir1.addDatapoint(75, 8.5);
  ir1.addDatapoint(93, 8);
  ir1.addDatapoint(101, 7.5);
  ir1.addDatapoint(113, 7);
  ir1.addDatapoint(129, 6.5);
  ir1.addDatapoint(147, 6);
  ir1.addDatapoint(165, 5.5);
  ir1.addDatapoint(195, 5);
  ir1.addDatapoint(232, 4.5);
  ir1.addDatapoint(269, 4);
  ir1.addDatapoint(310, 3.5);
  ir1.addDatapoint(370, 3);
  ir1.addDatapoint(500, 2.5);

  //ir2.addDatapoint(25, 12);
  //ir2.addDatapoint(38, 11.5);
  //ir2.addDatapoint(44, 11);
  //ir2.addDatapoint(50, 10.5);
  //ir2.addDatapoint(62, 10);
  //ir2.addDatapoint(68, 9.5);
  ir2.addDatapoint(78, 9);
  ir2.addDatapoint(93, 8.5);
  ir2.addDatapoint(105, 8);
  ir2.addDatapoint(112, 7.5);
  ir2.addDatapoint(125, 7);
  ir2.addDatapoint(142, 6.5);
  ir2.addDatapoint(155, 6);
  ir2.addDatapoint(179, 5.5);
  ir2.addDatapoint(193, 5);
  ir2.addDatapoint(219, 4.5);
  ir2.addDatapoint(254, 4);
  ir2.addDatapoint(320, 3.5);
  ir2.addDatapoint(386, 3);
  ir2.addDatapoint(510, 2.5);

  //ir3.addDatapoint(14, 12);
  //ir3.addDatapoint(19, 11.5);
  //ir3.addDatapoint(22, 11);
  //ir3.addDatapoint(26, 10.5);
  //ir3.addDatapoint(38, 10);
  //ir3.addDatapoint(48, 9.5);
  ir3.addDatapoint(58, 9);
  ir3.addDatapoint(70, 8.5);
  ir3.addDatapoint(83, 8);
  ir3.addDatapoint(95, 7.5);
  ir3.addDatapoint(108, 7);
  ir3.addDatapoint(125, 6.5);
  ir3.addDatapoint(140, 6);
  ir3.addDatapoint(157, 5.5);
  ir3.addDatapoint(182, 5);
  ir3.addDatapoint(212, 4.5);
  ir3.addDatapoint(244, 4);
  ir3.addDatapoint(290, 3.5);
  ir3.addDatapoint(352, 3);
  ir3.addDatapoint(460, 2.5);

  imu.init();

  //delay(2000);

  hal.init();  

  

}

void loop() {

  // Do HLTM
  hal.update();

  // Do position update
  imu.update();
  
  // Stepper update
  Stepper::updateAll();

  // LED Update
  LED::updateAll();
  
  // Do debug output
  debug();
  
}

unsigned long lastStatusPing = 0;

void debug() {

  /*Serial.print("P ");
  printDouble(imu.pitch, 100);
  Serial.print(" Y ");
  printDouble(imu.yaw, 100);
  /*Serial.print(" R ");
  printDouble(imu.roll, 100);
  Serial.print(" H ");
  printDouble(imu.heading, 100);
  Serial.println();

  Serial.print("C ");
  printDouble(imu.mag[0], 100);
  Serial.print(" C ");
  printDouble(imu.mag[1], 100);
  Serial.print(" C ");
  printDouble(imu.mag[2], 100);
  Serial.println();

  /*Serial.print("G ");
  printDouble(imu.gyro[0], 100);
  Serial.print(" G ");
  printDouble(imu.gyro[1], 100);
  Serial.print(" G ");
  printDouble(imu.gyro[2], 100);
  Serial.print(" H ");
  Serial.println();*/

  Serial.println();

  /*Serial.print(as0.analogReadSmooth(ir0));
  Serial.print(" ");
  Serial.print(as1.analogReadSmooth(ir1));
  Serial.print(" ");
  Serial.print(as2.analogReadSmooth(ir2));
  Serial.print(" ");
  Serial.println(as3.analogReadSmooth(ir3));*/
  
  if(state == IDLE && (micros() - lastStatusPing) > 500000) {
    
    led.off(LED::GREEN);
    led.off(LED::BLUE);
    led.blink(LED::RED, 250000);
    lastStatusPing = micros();
    
  } else if(state == RUNNING && (micros() - lastStatusPing) > 500000) {
    
    led.off(LED::RED);
    led.off(LED::BLUE);
    led.blink(LED::GREEN, 250000);
    lastStatusPing = micros();
    
  } else if(state == ERROR) {
    
    led.off(LED::GREEN);
    led.off(LED::BLUE);
    led.solid(LED::RED);
    
  } else if(state == COMPLETE) {
    
    led.off(LED::GREEN);
    led.off(LED::RED);
    led.solid(LED::BLUE);
    lastStatusPing = micros();
    
  }
  
}

