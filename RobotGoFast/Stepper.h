#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>

class Stepper {

public:

  enum Direction { FORWARD, BACKWARD };

  static const int enablePin;
  static const double stepsPerRevolution;
  static const double microsteps;
  static const double wheelDiameter; // In inches
  static const double wheelSpacing; // In inches
  static const unsigned long maxSpeed;
  
private:

  const int stepPin;
  const int dirPin;
  const bool reversed;
  unsigned long lastUpdateTime; // In uS
  unsigned long stepCount; // In microsteps
  unsigned long delay; // In uS
  double speed; // In Inches / Second
  
  Direction direction;
  
  // Global stepper object pointer array, needed by the update static method
  static Stepper * steppers[2];

  static bool enabled;

public:

  // Constructors
  Stepper(int stepPin, int dirPin, bool reversed);

  // Methods
  void set(double speed, Direction direction);
  void stop();
  unsigned long getCount();
  void resetCount();

  // Statics
  static void enableAll();
  static void disableAll();
  static void updateAll();

private:

  void update();

};

#endif // STEPPER_H
