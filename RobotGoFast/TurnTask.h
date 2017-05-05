#ifndef TURNTASK_H
#define TURNTASK_H

#include "Task.h"
#include "Stepper.h"
#include "PID.h"
#include "Vec2.h"

extern Stepper leftMotor;
extern Stepper rightMotor;

extern Backtracking backtracking;

class TurnTask : public Task {

private:

  float deg;
  long steps;
  PID pid;
  bool direction = true;

public:

  // Constructors
  TurnTask();
  TurnTask(Task * parent, float deg, String name);

  // Methods
  int update() override;
  void init() override;

  // Statics
  static bool between(float value, float low, float high);
  
};

#endif // TURNTASK_H
