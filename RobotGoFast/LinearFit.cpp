#include "LinearFit.h"

const float LinearFit::TOO_CLOSE = 0;
const float LinearFit::TOO_FAR = 0xFFFF;

LinearFit::LinearFit(int pin) : pin(pin) {
  numDatapoints = 0;
  pinMode(pin, INPUT);
}

void LinearFit::addDatapoint(float x, float y) {
  this->x[numDatapoints] = x;
  this->y[numDatapoints] = y;
  numDatapoints++;
}

float LinearFit::read() {
  
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
  
  //SerialUSB.print("raw ");
  return TOO_CLOSE;
  
}

float LinearFit::lerp(float x, float x0, float y0, float x1, float y1) {

  return y0 + (x - x0) * ((y1 - y0) / (x1 - x0));
  
}
