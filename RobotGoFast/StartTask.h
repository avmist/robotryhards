#ifndef STARTTASK_H
#define STARTTASK_H

#include "Task.h"

class StartTask : public Task {

private:

public:

  // Constructors
  StartTask(Task * parent, String name);
  StartTask(Task * mom, Task * dad, String name);

  // Methods
  bool update() override;
  void init() override;
  
};

#endif // STARTTASK_H
