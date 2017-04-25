#include "LED.h"

// Declare statics, see http://bit.ly/2oRM8vK
LED * LED::leds[10];

LED::LED(int rPin, int gPin, int bPin) {

  pins[0] = rPin;
  pins[1] = gPin;
  pins[2] = bPin;
  
  duration[0] = duration[1] = duration[2] = 0;
  on[0] = on[1] = on[2] = false;
  states[0] = states[1] = states[2] = LED::OFF;

  // Add to LED index array
  static int ledId = 0;
  LED::leds[ledId++] = this;

  // Set pin modes  
  pinMode(pins[0], OUTPUT);
  pinMode(pins[1], OUTPUT);
  pinMode(pins[2], OUTPUT);

  // Initial Values
  analogWrite(pins[0], 1023);
  analogWrite(pins[1], 1023);
  analogWrite(pins[2], 1023);
  
}

void LED::update() {

  // Time calculations
  unsigned long t = micros();
  unsigned long dt[3];
  
  dt[0] = t - lastUpdateTime[0];
  dt[1] = t - lastUpdateTime[1];
  dt[2] = t - lastUpdateTime[2];

  // Red
  if (states[LED::RED] == LED::BLINKING && on[LED::RED]) {

    // Check duration
    if (dt[LED::RED] > duration[LED::RED]) {
      toggle(LED::RED);
      lastUpdateTime[LED::RED] = t;
    }
    
  }
  
  // Green
  if (states[LED::GREEN] == LED::BLINKING && on[LED::GREEN]) {

    // Check duration
    if (dt[LED::GREEN] > duration[LED::GREEN]) {
      toggle(LED::GREEN);
      lastUpdateTime[LED::GREEN] = t;
    }
    
  }

  // Blue
  if (states[LED::BLUE] == LED::BLINKING && on[LED::BLUE]) {

    // Check duration
    if (dt[LED::BLUE] > duration[LED::BLUE]) {
      toggle(LED::BLUE);
      lastUpdateTime[LED::BLUE] = t;
    }
    
  }
  
}

void LED::updateAll() {

  leds[0]->update();
  
}

void LED::toggle(Color color) {

  if (on[color]) {
    on[color] = false;
    states[color] = LED::OFF;
    analogWrite(pins[color], 1023);
  } else {
    on[color] = true;
    analogWrite(pins[color], 512);
  }
  
}

void LED::blink(Color color, unsigned long dur) {

  states[color] = LED::BLINKING;

  if(!on[color]) {
    toggle(color);
  }
  
  duration[color] = dur;
  lastUpdateTime[color] = micros();
}

void LED::solid(Color color) {

  states[color] = LED::SOLID;

  if(!on[color]) {
    toggle(color);
  }
  
}

void LED::off(Color color) {

  states[color] = LED::OFF;

  if(on[color]) {
    toggle(color);
  }
  
}
