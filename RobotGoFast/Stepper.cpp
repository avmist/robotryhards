#include "Stepper.h"

// Declare statics, see http://bit.ly/2oRM8vK
Stepper * Stepper::steppers[2];
bool Stepper::enabled = false;
const int Stepper::enablePin = 4;
const float Stepper::stepsPerRevolution = 200;
const float Stepper::microsteps = 8;
const float Stepper::wheelDiameter = 2.786; // In Inches
 const float Stepper::wheelSpacing = 4.491; // In inches
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

  int leftStesps = steppers[0]->update();
  //Serial.println(leftStesps);
  int rightSteps = steppers[1]->update();
  //Serial.println(rightSteps);

  //while(leftStesps || rightSteps) {

      // Step Left if needed
      if(leftStesps) {
        digitalWrite(steppers[0]->stepPin, HIGH);
        leftStesps--;
      }

      // Step Right if needed
      if(rightSteps) {
        digitalWrite(steppers[1]->stepPin, HIGH);
        rightSteps--;
      }

      // Delay 2 uS
      delayMicroseconds(2);

      // Set both low
      digitalWrite(steppers[0]->stepPin, LOW);
      digitalWrite(steppers[1]->stepPin, LOW);

      steppers[0]->stepCount += 1;
      steppers[1]->stepCount += 1;

      // Delay 2 uS
      delayMicroseconds(2);

  //}
  
}

// Methods
int Stepper::update() {

  // Time calculations
  unsigned long t = micros();
  unsigned long dt = t - lastUpdateTime;

  if(dt > delay) {
    
    lastUpdateTime = t;

    return (int) (dt / delay);
    
  }

  return 0;
  
}

void Stepper::set(float speed, Direction direction) {

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
  float rpm = 60 * this->speed / (PI * wheelDiameter);
  //printDouble(rpm, 100000);
  
  float stepsPerMinute = rpm * stepsPerRevolution * microsteps;
  //printDouble(stepsPerMinute, 100000);
  
  float secondsPerStep = 60.0f / stepsPerMinute;
  //printDouble(secondsPerStep, 100000);
  
  this->delay = secondsPerStep * 1000000.0f;
  
  //Serial.println((int) this->delay);
  
}

void Stepper::stop() {
  this->delay = 0;
}

long Stepper::getCount() {
  long count = stepCount;
  resetCount();
  return count;
}

void Stepper::resetCount() {
  stepCount = 0;
}
