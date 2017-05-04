#ifndef DELAYTASK_H
#define DELAYTASK_H

#include "Task.h"

class DelayTask : public Task {

private:

  unsigned long delay;
  unsigned long startTime;

public:

  // Constructors
  DelayTask(Task * parent, unsigned long delay, String name);

  // Methods
  int update() override;
  void init() override;
  
};

#endif // DELAYTASK_H
