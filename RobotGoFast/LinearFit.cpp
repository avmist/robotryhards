#include "LinearFit.h"

const double LinearFit::TOO_CLOSE = 0;
const double LinearFit::TOO_FAR = 0xFFFF;

LinearFit::LinearFit(int pin) : pin(pin) {
  numDatapoints = 0;
  pinMode(pin, INPUT);
}

void LinearFit::addDatapoint(double x, double y) {
  this->x[numDatapoints] = x;
  this->y[numDatapoints] = y;
  numDatapoints++;
}

double LinearFit::read() {
  
  int raw = analogRead(pin);

  int lowPoint, highPoint;

  if(x[0] > raw) {
    return TOO_FAR;
  }
  
  for(int i = 1; i < numDatapoints; ++i) {
    
    if(x[i] >= raw) {
      
      highPoint = i;
      lowPoint = i - 1;
      
      return lerp(raw, x[lowPoint], y[lowPoint], x[highPoint], y[highPoint]);
      
    }
    
  }
  
  //Serial.print("raw ");
  return TOO_CLOSE;
  
}

double LinearFit::lerp(double x, double x0, double y0, double x1, double y1) {

  return y0 + (x - x0) * ((y1 - y0) / (x1 - x0));
  
}
