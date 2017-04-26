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

#include "PID.h"

PID::PID(double Kp, double Ki, double Kd, double min, double max) {
	
  outMin = min;
  outMax = max;

  this->kp = Kp;
  this->ki = Ki;
  this->kd = Kd;

  lastTime = millis();

  //double SampleTimeInSec = ((double) SampleTime) / 1000;  
  //kp = Kp;
  //ki = Ki * SampleTimeInSec;
  //kd = Kd / SampleTimeInSec;

}

double PID::Compute(double input, double setpoint) {

  const unsigned long t = millis();

  const double dT = (t - lastTime) / 1000000.f; // In seconds

  const double error = setpoint - input;

  errorSum += (error * dT);

  const double dInput = (input - lastInput);

  //const double dErr = (error - lastErr) / dT;

  const double output = (kp * error) + (ki * errorSum) - (kd * dInput);

  lastInput = input;
  lastError = error;
  lastTime = t;

  if(output > outMax) {
    return outMax;
  } else if(output < outMin) {
    return outMin;
  } else {
    return output;
  }

}