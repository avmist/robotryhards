#include "Adafruit_VL6180X.h"
#include "Stepper.h"
#include "Task.h"
#include "RobotState.h"
#include "LED.h"
#include "LinearFit.h"
#include "TurnTask.h"
#include "DelayTask.h"
#include "StartTask.h"
#include "StopTask.h"
#include "GoTask2.h"
#include "HLTM.h"
#include "math.h"

/* ST    P2    X  X  X  X  X  X  X
 *    X  X     X  X  X  X  X  X  X
 *    X  X  J2    P5       X  X  X
 *    X  X  P3 X  X  X     X  X  X
 * P1       J1    P4       X  X  X
 * X  X  X     X  X  X J3        X   
 * X  X  X     X  P6       X  P7 X
 * X  X  X     X     X  X  X     X
 * X  X  X           X  X  X     X
 * X  X  X  X  X  X  X  X  X  ED X
 */

// Globals
LinearFit ir0(A0);
LinearFit ir1(A1);
LinearFit ir2(A2);
LinearFit ir3(A3);

enum RobotState state = IDLE;

Backtracking backtracking = NOT_BACKTRACKING;

Stepper leftMotor(1, 0, false);
Stepper rightMotor(3, 2, true);

LED led(13, 12, 11);

Adafruit_VL6180X vl = Adafruit_VL6180X();

// Task tree
StartTask go(NULL, "Start");

// P7
GoTask2 p7_start(NULL, 24, "Go 24 South");
TurnTask p7_1(&p7_start, 90, "Turn right 90 deg");
GoTask2 p7_2(&p7_1, 50, "Go 50 South");
StopTask p7_end(&p7_2, "End");

// // P4
GoTask2 p4_start(NULL, 52, "Go 52 East");
TurnTask p4_1(&p4_start, 90, "Turn right 90 deg");
GoTask2 p4_2(&p4_1, 13, "Go 13 South", true);
TurnTask p4_end(&p4_2, -90, "Turn left 90 deg");

// P1
GoTask2 p1_start(NULL, 50, "Go 50 South");
TurnTask p1_1(&p1_start, -90, "Turn left 90 deg"); // P1 Back jumping point
GoTask2 p1_end(&p1_1, 35, "Go 35 East");

// P6
TurnTask p6_start(NULL, 90, "Turn right 90 deg");
GoTask2 p6_1(&p6_start, 52, "Go 52 South");
TurnTask p6_2(&p6_1, -90, "Turn left 90 deg");
GoTask2 p6_3(&p6_2, 24, "Go 24 East");
TurnTask p6_4(&p6_3, -90, "Turn left 90 deg");
GoTask2 p6_5(&p6_4, 24, "Go 24 North");
TurnTask p6_6(&p6_5, 90, "Turn right 90 deg");
GoTask2 p6_7(&p6_6, 24, "Go 24 East");
TurnTask p6_8(&p6_7, -90, "Turn left 90 deg");
GoTask2 p6_9(&p6_8, 12, "Go 12 North");
TurnTask p6_end(&p6_9, 90, "Turn right 90 deg");

// P5
GoTask2 p5_start(NULL, 52, "Go 52 East");
TurnTask p5_1(&p5_start, 90, "Turn right 90 deg");
GoTask2 p5_2(&p5_1, 23, "Go 23 South", true);
TurnTask p5_end(&p5_2, -90, "Turn left 90 deg");

// P3 Up
TurnTask p3u_start(NULL, -90, "Turn left 90 deg");
GoTask2 p3u_1(&p3u_start, 24, "Go 24 North");
TurnTask p3u_end(&p3u_1, 90, "Turn right 90 deg");

// // P3 Down
TurnTask p3d_start(NULL, 90, "Turn right 90 deg");
GoTask2 p3d_1(&p3d_start, 24, "Go 24 South");
TurnTask p3d_end(&p3d_1, -90, "Turn left 90 deg");

// P2
TurnTask p2_start(NULL, -90, "Turn left 90 deg");
GoTask2 p2_1(&p2_start, 36, "Go 36 East");
TurnTask p2_2(&p2_1, 105, "Turn right 105 deg");
GoTask2 p2_3(&p2_2, 24, "Go 24 South");
TurnTask p2_end(&p2_3, -90, "Turn left 90 deg");

HLTM hal(&go);

void TC4_Handler() {

  // Check for overflow (OVF) interrupt 
  if(TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF) {
    REG_TC4_INTFLAG = TC_INTFLAG_OVF;         // Clear the OVF interrupt flag 
  }

  Stepper::updateAll();

}

void setup() {

  analogWriteResolution(10);

  // Inital paths
  p1_start.setParent(&go);
  p2_start.setParent(&go);

  // J1
  p5_start.setParent(&p2_end);
  p3d_start.setParent(&p2_end);

  p3u_end.addChild(&p5_start);

  // J2
  p4_start.setParent(&p1_end);
  p3u_start.setParent(&p1_end);
  p6_start.setParent(&p1_end);
  
  p3d_end.addChild(&p4_start);
  p3d_end.addChild(&p6_start);
  
  // J3
  p4_end.addChild(&p7_start);
  p5_end.addChild(&p7_start);
  p6_end.addChild(&p7_start);

  SerialUSB.begin(115200);

  // From Adafruit example
  // wait for serial port to open on native usb devices
  while(!Serial) {
    delay(1);
  }

  //SerialUSB.print("Setup start...");

  if(!vl.begin()) {
    SerialUSB.println("Failed to connect to ToF.");
    while(1);
  }

  pinMode(Stepper::enablePin, OUTPUT);

  // Disable steppers initially
  digitalWrite(Stepper::enablePin, HIGH);

  // Timer stuff
  // Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_TC4_COUNT16_CC0 = 150;                      // Set the TC4 CC0 register as the TOP value in match frequency mode
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization

  //NVIC_DisableIRQ(TC4_IRQn);
  //NVIC_ClearPendingIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  REG_TC4_INTFLAG |= TC_INTFLAG_OVF;              // Clear the interrupt flags
  REG_TC4_INTENSET = TC_INTENSET_OVF;             // Enable TC4 interrupts
  // REG_TC4_INTENCLR = TC_INTENCLR_OVF;          // Disable TC4 interrupts

  REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV256 |   // Set prescaler to 1024, 48MHz/1024 = 46.875kHz
                   TC_CTRLA_WAVEGEN_MFRQ |        // Put the timer TC4 into match frequency (MFRQ) mode 
                   TC_CTRLA_ENABLE;               // Enable TC4
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization
  
  ir0.addDatapoint(19, 12);
  ir0.addDatapoint(22, 11.5);
  ir0.addDatapoint(27, 11);
  ir0.addDatapoint(33, 10.5);
  ir0.addDatapoint(45, 10);
  ir0.addDatapoint(52, 9.5);
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

  ir1.addDatapoint(16, 12);
  ir1.addDatapoint(22, 11.5);
  ir1.addDatapoint(25, 11);
  ir1.addDatapoint(33, 10.5);
  ir1.addDatapoint(39, 10);
  ir1.addDatapoint(51, 9.5);
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

  ir2.addDatapoint(25, 12);
  ir2.addDatapoint(38, 11.5);
  ir2.addDatapoint(44, 11);
  ir2.addDatapoint(50, 10.5);
  ir2.addDatapoint(62, 10);
  ir2.addDatapoint(68, 9.5);
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

  ir3.addDatapoint(14, 12);
  ir3.addDatapoint(19, 11.5);
  ir3.addDatapoint(22, 11);
  ir3.addDatapoint(26, 10.5);
  ir3.addDatapoint(38, 10);
  ir3.addDatapoint(48, 9.5);
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

  hal.init();

  //SerialUSB.println("   Complete.");

}

void loop() {

  // Do debug output
  debug();

  // Do HLTM
  hal.update();
  
  // Stepper update
  //Stepper::updateAll();

  // LED Update
  LED::updateAll();
  
}

unsigned long lastStatusPing = 0;

void debug() {

  static int foo = 0;

  // if(foo == 0) {
  //   delay(8000);
  //   go.print(0);
  //   foo = 1;
  // }
  
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
