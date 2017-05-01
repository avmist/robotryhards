// Code from https://github.com/br3ttb/Arduino-PID-Library
// We (embedded systems group @ CU Denver) make absolutely NO CLAIMS to this
// work, we did not create this particular part of this assignment nor do
// we consider or claim this to be our own work.
// Consider this an official source of external work that we have integrated
// into our project in an effort to not reinvent the wheel, like good
// computer science students are taught to do.
// Some adaptation was made

/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID {

  public:

    PID(float Kp, float Ki, float Kd, float min, float max);
    PID();
	
    float Compute(float input, float setpoint);

  private:

	float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter

	unsigned long lastTime;
	float errorSum, lastInput, lastError;

	float outMin, outMax;

};

#endif // PID_H
