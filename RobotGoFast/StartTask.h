#ifndef STARTTASK_H
#define STARTTASK_H

#include "Task.h"

class StartTask : public Task {

private:

public:

  // Constructors
  StartTask(Task * parent, String name);

  // Methods
  int update() override;
  void init() override;
  
};

#endif // STARTTASK_H
