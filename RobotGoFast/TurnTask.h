#ifndef TURNTASK_H
#define TURNTASK_H

#include "Task.h"
#include "Stepper.h"
#include "PID.h"

extern Stepper leftMotor;
extern Stepper rightMotor;

class TurnTask : public Task {

private:

  int deg;
  unsigned long steps;
  PID pid;
  double initialHeading;

public:

  // Constructors
  TurnTask(Task * parent, double deg, String name);
  TurnTask(Task * mom, Task * dad, double deg, String name);

  // Methods
  bool update() override;
  void init() override;
  
};

#endif // TURNTASK_H
