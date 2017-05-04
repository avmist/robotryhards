#ifndef STOPTASK_H
#define STOPTASK_H

#include "Task.h"

class StopTask : public Task {

private:

public:

  // Constructors
  StopTask(Task * parent, String name);
  
  // Methods
  int update() override;
  void init() override;
  
};

#endif // STOPTASK_H
