#include <AnalogSmooth.h>

const double wheelSpacing = 4.268; // In inches
enum RobotState { IDLE, RUNNING, ERROR };
enum RobotState state = IDLE;

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

   Serial.println(frac,DEC);
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
  unsigned long getCount();
  void resetCount();

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
  printDouble(rpm, 100000);
  
  double stepsPerMinute = rpm * stepsPerRevolution * microsteps;
  printDouble(stepsPerMinute, 100000);
  
  double secondsPerStep = 60.0f / stepsPerMinute;
  printDouble(secondsPerStep, 100000);
  
  this->delay = secondsPerStep * 1000000.0f;
  
  printDouble(this->delay, 100000);
  
}

unsigned long Stepper::getCount() {
  return this->stepCount;
}

void Stepper::resetCount() {
  this->stepCount = 0;
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
// Task
// ====================================================

class Task {

public:

protected:

  Task * parents[2];
  int numParents;
  Task * children[2];
  int numChildren;
  static Task * currentTask;
  bool traversed;
  bool finished;
  
public:

  // Constructors
  Task(Task * parent);
  Task(Task * mom, Task * dad);

  // Methods
  //virtual void update();
  bool update();
  Task * getUntreversedChild();

private:

  // Methods
  void addChild(Task * task);
  void addParent(Task * task);
  
};

Task::Task(Task * parent) {
   
  numParents = 0;
  numChildren = 0;
  
  // Add to parents list
  this->addParent(parent);
  
  // Add to parents children list
  this->parents[0]->addChild(this);

  traversed = false;
  finished = false;
  
}

Task::Task(Task * mom, Task * dad) {

  numParents = 0;
  numChildren = 0;

  // Add to parents list
  this->addParent(mom);
  this->addParent(dad);

  // Add to children list
  this->parents[0]->addChild(this);
  this->parents[1]->addChild(this);

  traversed = false;
  
}

void Task::addChild(Task * task) {
  children[numChildren++] = task;
}

void Task::addParent(Task * task) {
  parents[numParents++] = task;
}

bool Task::update() {
  
}

Task * Task::getUntreversedChild() {

  for(int i = 0; i < numChildren; ++i) {

    if(children[i]->traversed) {
      return children[i];
    }
    
  }

  return NULL;
  
}

// ====================================================
// TurnTask
// ====================================================

class TurnTask : public Task {

private:

  int deg;

public:

  // Constructors
  TurnTask(Task * parent, int deg);
  TurnTask(Task * mom, Task * dad, int deg);

  // Methods
  bool update();
  
};

TurnTask::TurnTask(Task * parent, int deg) : Task(parent) {
  this->deg = deg;
}

TurnTask::TurnTask(Task * mom, Task * dad, int deg) : Task(mom, dad) {
  this->deg = deg;
}

bool TurnTask::update() {

}

// ====================================================
// StartTask
// ====================================================

class StartTask : public Task {

private:

public:

  // Constructors
  StartTask(Task * parent);
  StartTask(Task * mom, Task * dad);

  // Methods
  bool update();
  
};

StartTask::StartTask(Task * parent) : Task(parent){
  Stepper::enableAll();
  state = RUNNING;
}

StartTask::StartTask(Task * mom, Task * dad) : Task(mom, dad){
  Stepper::enableAll();
  state = RUNNING;
}

bool StartTask::update() {
  
}

// ====================================================
// StopTask
// ====================================================

class StopTask : public Task {

private:

public:

  // Constructors
  StopTask(Task * parent);
  StopTask(Task * mom, Task * dad);

  // Methods
  bool update();
  
};

StopTask::StopTask(Task * parent) : Task(parent) {
  Stepper::disableAll();
  state = IDLE;
}

StopTask::StopTask(Task * mom, Task * dad) : Task(mom, dad) {
  Stepper::disableAll();
  state = IDLE;
}

bool StopTask::update() {
  
}

// ====================================================
// Task tree
// ====================================================

/*
 * S = Start
 * E = End
 * . = Junction
 *   = Traversable
 * X = Not Traversable
 * 
 * Starting Orientation
 *  |
 * \ /
 * 
 * ST    P2    X  X  X  X  X  X
 *    X  X     X  X  X  X  X  X
 *    X  X  J2    P5       X  X
 *    X  X  P3 X  X  X     X  X
 * P1       J1    P4       X  X
 * X  X  X  X  X  X  X  J3        
 * X  X  X  X  X  X  X  X  X   
 * X  X  X  X  X  X  X  X  X  P6
 * X  X  X  X  X  X  X  X  X   
 * X  X  X  X  X  X  X  X  X  ED
 */
 
// Chain 1 Start @ Start
/*Task root(NULL, Task::START);
Task t1_1(&root, Task::FORWARD);
Task t1_2(&t1_1, Task::FORWARD);
Task t1_3(&t1_2, Task::FORWARD);
Task t1_4(&t1_3, Task::FORWARD);
TurnTask t1_5(&t1_4, -90);
Task t1_6(&t1_5, Task::FORWARD);
Task t1_7(&t1_6, Task::FORWARD);
Task t1_8(&t1_7, Task::FORWARD);
// Chain 1 End @ J1

// Chain 2 Start @ Start
TurnTask t2_1(&root, -90;
Task t2_2(&t2_1, Task::FORWARD);
Task t2_3(&t2_2, Task::FORWARD);
Task t2_4(&t2_3, Task::FORWARD);
Task t2_5(&t2_4, Task::RIGHT);
Task t2_6(&t2_5, Task::FORWARD);
Task t2_7(&t2_6, Task::FORWARD);
Task t2_8(&t2_7, -90);
// Chain 2 End @ J2

// Chain 3a Start @ J1
Task t3a_1(&t1_8, -90);
Task t3a_2(&t3a_1, Task::FORWARD);
Task t3a_3(&t3a_2, Task::FORWARD);
Task t3a_4(&t3a_4, 90);
// Chain 3a End @ J2

// Chain 3b Start @ J2
Task t3b_1(&t2_8, 90);
Task t3b_2(&t3b_1, Task::FORWARD);
Task t3b_3(&t3b_2, Task::FORWARD);
Task t3b_4(&t3b_3, -90);
// Chain 3b End @ J1

// Chain 4 Start @ J1
Task t4_1(&t1_8, Task::FORWARD);
Task t4_2(&t4_1, Task::FORWARD);
Task t4_3(&t4_2, Task::FORWARD);
Task t4_4(&t4_3, Task::FORWARD);
Task t4_5(&t4_4, 90);
Task t4_6(&t4_5, Task::FORWARD);
// Chain 4 End @ J3

// Chain 5 Start @ J2
Task t5_1(&t2_8, Task::FORWARD);
Task t5_2(&t5_1, Task::FORWARD);
Task t5_3(&t5_2, Task::FORWARD);
Task t5_4(&t5_3, Task::FORWARD);
Task t5_5(&t5_4, 90);
Task t5_6(&t5_5, Task::FORWARD);
Task t5_7(&t5_6, Task::FORWARD);
Task t5_8(&t5_7, Task::FORWARD);
// Chain 5 End @ J3

// Chain 6 Start @ J3
Task t6_1(&t5_8, Task::LEFT);
Task t6_2(&t6_1, Task::FORWARD);
Task t6_3(&t6_2, Task::FORWARD);
Task t6_4(&t6_3, Task::RIGHT);
Task t6_5(&t6_4, Task::FORWARD);
Task t6_6(&t6_5, Task::FORWARD);
Task t6_7(&t6_6, Task::FORWARD);
Task t6_8(&t6_7, Task::FORWARD);
Task t6_9(&t6_8, Task::STOP);*/
// Chain 5 End @ End

// Turn test
StartTask turn(NULL);
TurnTask turn1(&turn, 90);
StopTask turn2(&turn1);

// ====================================================
// HLTM
// ====================================================

class HLTM {

public:

private:
  
  Task * rootTask;
  Task * currentTask;
  
public:
  
  // Constructors
  HLTM(Task * rootTask);
  
  // Statics

  // Methods
  void update();
  
private:
  
};

HLTM::HLTM(Task * rootTask) {
  this->rootTask = rootTask;
  this->currentTask = rootTask;
}

void HLTM::update() {

  if(currentTask == NULL) {
    state = ERROR;
    return;
  }
  
  if(currentTask->update()) {
    currentTask = currentTask->getUntreversedChild();
  }
  
}


// ====================================================
// Main Loop
// ====================================================

// Globals
const int ir0 = A0;
const int ir1 = A1;
const int ir2 = A2;
const int ir3 = A3;

Stepper leftMotor(1, 0, false);
Stepper rightMotor(3, 2, true);
LED led(13, 12, 11);
HLTM hal(&turn);

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

  //
  
}

void loop() {

  // Do HLTM
  hal.update();
  
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
  } else if(state == RUNNING) {
    led.solid(LED::RED);
  }
  
}

