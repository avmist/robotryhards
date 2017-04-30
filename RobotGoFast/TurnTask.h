#ifndef TURNTASK_H
#define TURNTASK_H

#include "Task.h"
#include "Stepper.h"
#include "PID.h"
#include "IMU.h"
#include "Vec2.h"

extern IMU imu;
extern Stepper leftMotor;
extern Stepper rightMotor;

class TurnTask : public Task {

private:

  float deg;
  long steps;
  PID pid;
  Vec2 initialVec, finalVec;
  int samples = 50;
  bool direction = true;

public:

  // Constructors
  TurnTask(Task * parent, float deg, String name);
  TurnTask(Task * mom, Task * dad, float deg, String name);

  // Methods
  bool update() override;
  void init() override;

  // Statics
  static bool between(float value, float low, float high);
  
};

#endif // TURNTASK_H
