#ifndef GOTASK_H
#define GOTASK_H

#include "Task.h"
#include "LinearFit.h"
#include "RobotState.h"
#include "LED.h"
#include "Vec2.h"

extern Stepper leftMotor;
extern Stepper rightMotor;

extern LED led;

extern LinearFit ir0;
extern LinearFit ir1;
extern LinearFit ir2;
extern LinearFit ir3;

class GoTask : public Task {

private:

  int distance;
  unsigned long steps;
  unsigned long lastStatusPing;

public:

  // Constructors
  GoTask(Task * parent, double distance, String name);
  GoTask(Task * mom, Task * dad, double distance, String name);

  // Methods
  bool update() override;
  void init() override;
  
};

#endif // GOTASK_H
