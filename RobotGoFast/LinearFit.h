#ifndef LINEARFIT_H
#define LINEARFIT_H

#include <Arduino.h>

class LinearFit {

  // X = raw input
  // Y mapping

public:

  const static float TOO_CLOSE;
  const static float TOO_FAR;

private:

  float x[20];
  float y[20];
  int numDatapoints;
  const int pin;
  
public:
  
  // Constructors
  LinearFit(int pin);
  
  // Statics
  static float lerp(float x, float x0, float y0, float x1, float y1);

  // Methods
  void addDatapoint(float x, float y);    // Needs to be monotonic on x
                                            // Two points minimum
  float read();

private:
  
};

#endif // LINEARFIT_H
