#ifndef LED_H
#define LED_H

#include <Arduino.h>

class LED {

public:

  enum Color { RED = 0, GREEN = 1, BLUE = 2 };
  enum State { SOLID, BLINKING, OFF };

private:

  int pins[3];
  State states[3];
  unsigned long duration[3];
  unsigned long lastUpdateTime[3]; // In uS
  bool on[3];

  // Global stepper object pointer array, needed by the update static method
  static LED * leds[10];

public:

  // Constructors
  LED(int rPin, int gPin, int bPin);

  // Statics
  static void updateAll();

  // Methods
  void blink(Color color, unsigned long dur);
  void solid(Color color);
  void off(Color color);

private:

  void update();
  void toggle(Color color);
  
};

#endif // LED_H
