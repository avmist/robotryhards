#ifndef GOTASK_H
#define GOTASK_H

#include "Task.h"

extern Stepper leftMotor;
extern Stepper rightMotor;

class GoTask : public Task {

private:

  int distance;
  unsigned long steps;

public:

  // Constructors
  GoTask(Task * parent, double distance, String name);
  GoTask(Task * mom, Task * dad, double distance, String name);

  // Methods
  bool update() override;
  void init() override;
  
};

#endif // GOTASK_H
