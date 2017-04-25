#ifndef LINEARFIT_H
#define LINEARFIT_H

#include <Arduino.h>

class LinearFit {

  // X = raw input
  // Y mapping

public:

  const static double TOO_CLOSE;
  const static double TOO_FAR;

private:

  double x[20];
  double y[20];
  int numDatapoints;
  const int pin;
  
public:
  
  // Constructors
  LinearFit(int pin);
  
  // Statics
  static double lerp(double x, double x0, double y0, double x1, double y1);

  // Methods
  void addDatapoint(double x, double y);    // Needs to be monotonic on x
                                            // Two points minimum
  double read();

private:
  
};

#endif // LINEARFIT_H
