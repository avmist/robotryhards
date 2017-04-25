#include "Stepper.h"

// Declare statics, see http://bit.ly/2oRM8vK
Stepper * Stepper::steppers[2];
bool Stepper::enabled = false;
const int Stepper::enablePin = 4;
const double Stepper::stepsPerRevolution = 200;
const double Stepper::microsteps = 8;
const double Stepper::wheelDiameter = 2.786; // In Inches
 const double Stepper::wheelSpacing = 4.491; // In inches
const unsigned long Stepper::maxSpeed = 21; // Inches / Second

// Constructors
Stepper::Stepper(int stepPin, int dirPin, bool reversed) : stepPin(stepPin), dirPin(dirPin), reversed(reversed) {

  // Initialize members
  stepCount = 0;
  speed = 0;
  direction = FORWARD;

  // Add to stepper index array
  static int stepperId = 0;
  Stepper::steppers[stepperId++] = this;

  // Set pin modes
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  // Initial Values
  digitalWrite(dirPin, LOW);
  digitalWrite(stepPin, LOW);

}

// Statics
void Stepper::enableAll() {
  digitalWrite(enablePin, LOW);
  enabled = true;
}

void Stepper::disableAll() {
  digitalWrite(enablePin, HIGH);
  enabled = false;
}

void Stepper::updateAll() {

  steppers[0]->update();
  steppers[1]->update();
  
}

// Methods
void Stepper::update() {

  // Time calculations
  unsigned long t = micros();
  unsigned long dt = t - lastUpdateTime;

  if(dt > delay) {

    // Step motor
    if(delay > 0) {

      // Don't step if the speed is set to zero, just reset the lastUpdateTime
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(2);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(2);
      stepCount++;
      
    }
    
    lastUpdateTime = t;
    
  }
  
}

void Stepper::set(double speed, Direction direction) {

  this->speed = fmin(speed, maxSpeed);
  this->direction = direction;

  if(reversed) {

    if(direction == Stepper::FORWARD) {
      digitalWrite(dirPin, HIGH);
    } else {
      digitalWrite(dirPin, LOW);
    }
    
  } else {

    if(direction == Stepper::FORWARD) {
      digitalWrite(dirPin, LOW);
    } else {
      digitalWrite(dirPin, HIGH);
    }
    
  }

  // Calculate step delay

  // Calculate speed as an RPM
  double rpm = 60 * this->speed / (PI * wheelDiameter);
  //printDouble(rpm, 100000);
  
  double stepsPerMinute = rpm * stepsPerRevolution * microsteps;
  //printDouble(stepsPerMinute, 100000);
  
  double secondsPerStep = 60.0f / stepsPerMinute;
  //printDouble(secondsPerStep, 100000);
  
  this->delay = secondsPerStep * 1000000.0f;
  
  //printDouble(this->delay, 100000);
  
}

void Stepper::stop() {
  this->delay = 0;
}

unsigned long Stepper::getCount() {
  return this->stepCount;
}

void Stepper::resetCount() {
  this->stepCount = 0;
}
