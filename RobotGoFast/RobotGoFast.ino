enum Direction { FORWARD, BACKWARD };

const float pie = 3.14159265359;

// ====================================================
// == Stepper
// ====================================================

class Stepper {

public:

  static const int enablePin;
  static const float stepsPerRevolution;
  static const float microsteps;
  static const float wheelDiameter; // In inches
  
private:

  const int stepPin;
  const int dirPin;
  const bool reversed;
  unsigned long lastUpdateTime; // In uS
  unsigned long stepCount; // In microsteps
  unsigned long delay; // In uS
  float speed; // In Inches / Second
  
  Direction direction;
  
  // Global stepper object pointer array, needed by the update static method
  static Stepper * steppers[2];

  static bool enabled;

public:

  // Constructors
  Stepper(int stepPin, int dirPin, bool reversed);

  // Methods
  void set(float speed, Direction direction);

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
const float Stepper::stepsPerRevolution = 200;
const float Stepper::microsteps = 8;
const float Stepper::wheelDiameter = 2.838; // In inches

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

  
  
  
}

void Stepper::set(float speed, Direction direction) {
  
  this->speed = speed;
  this->direction = direction;

  // Calculate step delay
  
  // Speed = Distance / Time (Inches / Second)
  
  // Degree per Step = 360 / (stepsPerRevolution / microsteps)
  float degPerSec = 360.f / (stepsPerRevolution / microsteps);
  
  // Distance Per Step = wheelDiameter * 2 * pie * (Degree per Step / 360)
  float distPerStep = wheelDiameter * 2.f * PI * (degPerSec / 360.f);
  
  // Delay = speed * (1 / Distance Per Step)
  this->delay = speed * (1 / distPerStep);
  
}

// ====================================================
// == LED
// ====================================================

class LED {

public:

private:

  const int rPin, gPin, bPin;
  unsigned long delay[3];
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
  void setRed(unsigned long del, unsigned long duration);
  void setGreen(unsigned long del, unsigned long duration);
  void setBlue(unsigned long del, unsigned long duration);

private:

  void update();
  void toggleRed();
  void toggleGreen();
  void toggleBlue();
  
};

// Declare statics, see http://bit.ly/2oRM8vK
LED * LED::leds[10];

LED::LED(int rPin, int gPin, int bPin) : rPin(rPin), gPin(gPin), bPin(bPin) {
  
  delay[0] = 0;
  delay[1] = 0;
  delay[2] = 0;
  duration[0] = 0;
  duration[1] = 0;
  duration[2] = 0;
  on[0] = false;
  on[1] = false;
  on[2] = false;

  // Add to LED index array
  static int ledId = 0;
  LED::leds[ledId++] = this;

  // Set pin modes  
  pinMode(rPin, OUTPUT);
  pinMode(gPin, OUTPUT);
  pinMode(bPin, OUTPUT);

  // Initial Values
  analogWrite(rPin, 1023);
  analogWrite(gPin, 1023);
  analogWrite(bPin, 1023);
  
}

void LED::update() {

  // Time calculations
  unsigned long t = micros();
  unsigned long dt[3];
  
  dt[0] = t - lastUpdateTime[0];
  dt[1] = t - lastUpdateTime[1];
  dt[2] = t - lastUpdateTime[2];

  /*Serial.print(dt[0]);
  Serial.print(" ");
  Serial.print(dt[1]);
  Serial.print(" ");
  Serial.println(dt[2]);

  Serial.print(lastUpdateTime[0]);
  Serial.print(" ");
  Serial.print(lastUpdateTime[1]);
  Serial.print(" ");
  Serial.println(lastUpdateTime[2]);

  
  
  Serial.print(delay[0]);
  Serial.print(" ");
  Serial.print(delay[1]);
  Serial.print(" ");
  Serial.println(delay[2]);

  Serial.print(duration[0]);
  Serial.print(" ");
  Serial.print(duration[1]);
  Serial.print(" ");
  Serial.println(duration[2]);
  Serial.println(on[0]);
  Serial.println();*/

  // Red
  if (on[0]) {

    // Check duration
    if (dt[0] > duration[0]) {
      toggleRed();
      lastUpdateTime[0] = t;
    }
    
  } else {

    // Check delay
    if (dt[0] > delay[0]) {
      toggleRed();
      lastUpdateTime[0] = t;
    }
    
  }

  // Green
  if (on[1]) {

    // Check duration
    if (dt[1] > duration[1]) {
      toggleGreen();
      lastUpdateTime[1] = t;
    }
    
  } else {

    // Check delay
    if (dt[1] > delay[1]) {
      toggleGreen();
      lastUpdateTime[1] = t;
    }
    
  }

  // Blue
  if (on[2]) {

    // Check duration
    if (dt[2] > duration[2]) {
      toggleBlue();
      lastUpdateTime[2] = t;
    }
    
  } else {

    // Check delay
    if (dt[2] > delay[2]) {
      toggleBlue();
      lastUpdateTime[2] = t;
    }
    
  }

  
  
  
}

void LED::updateAll() {

  leds[0]->update();
  
}

void LED::toggleRed() {

  //Serial.println("Toggle Red");

  if (on[0]) {
    on[0] = false;
    analogWrite(rPin, 1023);
  } else {
    on[0] = true;
    analogWrite(rPin, 0);
  }
  
}

void LED::toggleGreen() {

  //Serial.println("Toggle Green");

  if (on[1]) {
    on[1] = false;
    analogWrite(gPin, 1023);
  } else {
    on[1] = true;
    analogWrite(gPin, 0);
  }
  
}

void LED::toggleBlue() {

  //Serial.println("Toggle Blue");

  if (on[2]) {
    on[2] = false;
    analogWrite(bPin, 1023);
  } else {
    on[2] = true;
    analogWrite(bPin, 0);
  }
  
}

void LED::setRed(unsigned long del, unsigned long dur) {
  delay[0] = del;
  duration[0] = dur;
}

void LED::setGreen(unsigned long del, unsigned long dur) {
  delay[1] = del;
  duration[1] = dur;
}

void LED::setBlue(unsigned long del, unsigned long dur) {
  delay[2] = del;
  duration[2] = dur;
}






// ====================================================
// == Main Loop
// ====================================================

enum RobotState { IDLE, RUNNING_OK };

// Globals
const int ir0 = A0;
const int ir1 = A1;
const int ir2 = A2;
const int ir3 = A3;

Stepper leftMotor(0, 1, false);
Stepper rightMotor(2, 3, true);
LED led(11, 12, 13);

void setup() {

  analogWriteResolution(10);

  pinMode(Stepper::enablePin, OUTPUT);
  digitalWrite(Stepper::enablePin, HIGH);

  pinMode(ir0, INPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  
  Serial.begin(9600);

  led.setRed(500000, 500000);
  
}

void loop() {

  // Do HLTM

  // Do stepper update
  //Stepper::updateAll();

  LED::updateAll();

  // Do position update

  // Do debug output
  debug();
  
}

void debug() {

  Serial.print(analogRead(ir0));
  Serial.print(" ");
  Serial.print(analogRead(ir1));
  Serial.print(" ");
  Serial.print(analogRead(ir2));
  Serial.print(" ");
  Serial.println(analogRead(ir3));
  
}

