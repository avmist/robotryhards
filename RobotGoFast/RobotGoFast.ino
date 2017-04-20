#include <AnalogSmooth.h>

const double wheelSpacing = 4.268; // In inches

void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

   Serial.print (int(val));  //prints the int part
   Serial.print("."); // print the decimal point
   unsigned int frac;
   if(val >= 0)
     frac = (val - int(val)) * precision;
   else
      frac = (int(val)- val ) * precision;
   int frac1 = frac;
   while( frac1 /= 10 )
       precision /= 10;
   precision /= 10;
   while(  precision /= 10)
       Serial.print("0");

   Serial.println(frac,DEC) ;
}

// ====================================================
// Stepper
// ====================================================

class Stepper {

public:

  enum Direction { FORWARD, BACKWARD };

  static const int enablePin;
  static const double stepsPerRevolution;
  static const double microsteps;
  static const double wheelDiameter; // In inches
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

  // Statics
  static void enableAll();
  static void disableAll();
  static void updateAll();

private:

  void update();

};

// Declare statics, see http://bit.ly/2oRM8vK
Stepper * Stepper::steppers[2];
bool Stepper::enabled = false;
const int Stepper::enablePin = 4;
const double Stepper::stepsPerRevolution = 200;
const double Stepper::microsteps = 8;
const double Stepper::wheelDiameter = 2.838; // In Inches
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
  printDouble(rpm, 100000);
  
  double stepsPerMinute = rpm * stepsPerRevolution * microsteps;
  printDouble(stepsPerMinute, 100000);
  
  double secondsPerStep = 60.0f / stepsPerMinute;
  printDouble(secondsPerStep, 100000);
  
  this->delay = secondsPerStep * 1000000.0f;
  
  printDouble(this->delay, 100000);
  Serial.println();
  
}



// ====================================================
// LED
// ====================================================

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

// ====================================================
// Main Loop
// ====================================================

enum RobotState { IDLE, RUNNING };
enum RobotState state = IDLE;

// Globals
const int ir0 = A0;
const int ir1 = A1;
const int ir2 = A2;
const int ir3 = A3;

Stepper leftMotor(1, 0, false);
Stepper rightMotor(3, 2, true);
LED led(13, 12, 11);

/*AnalogSmooth as0 = AnalogSmooth();
AnalogSmooth as1 = AnalogSmooth();
AnalogSmooth as2 = AnalogSmooth();
AnalogSmooth as3 = AnalogSmooth();*/

void setup() {

  analogWriteResolution(10);

  pinMode(Stepper::enablePin, OUTPUT);
  digitalWrite(Stepper::enablePin, HIGH);

  pinMode(ir0, INPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  
  Serial.begin(9600);

  Stepper::enableAll();
  leftMotor.set(21, Stepper::FORWARD);
  rightMotor.set(21, Stepper::FORWARD);
  
}

void loop() {

  // Do HLTM

  // Stepper update
  Stepper::updateAll();

  // LED Update
  LED::updateAll();

  // Do position update

  // Do debug output
  debug();
  
}

unsigned long lastStatusPing = 0;

void debug() {

  /*Serial.print(as0.analogReadSmooth(ir0));
  Serial.print(" ");
  Serial.print(as1.analogReadSmooth(ir1));
  Serial.print(" ");
  Serial.print(as2.analogReadSmooth(ir2));
  Serial.print(" ");
  Serial.println(as3.analogReadSmooth(ir3));*/

  /*Serial.print(analogRead(ir0));
  Serial.print(" ");
  Serial.print(analogRead(ir1));
  Serial.print(" ");
  Serial.print(analogRead(ir2));
  Serial.print(" ");
  Serial.println(analogRead(ir3));*/

  if(state == IDLE && (micros() - lastStatusPing) > 500000) {
    led.blink(LED::GREEN, 250000);
    lastStatusPing = micros();
  } else if(state == RUNNING) {
    led.solid(LED::GREEN);
  }
  
}

