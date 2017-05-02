#ifndef HLTM_H
#define HLTM_H

#include "Task.h"
#include "TurnTask.h"
#include "DelayTask.h"
#include "StartTask.h"
#include "StopTask.h"
#include "GoTask.h"

class HLTM {

public:

private:

  Task * currentTask;
  
public:
  
  // Constructors
  HLTM(Task * rootTask);
  
  // Statics

  // Methods
  void update();
  void init();

private:
  
};

#endif // HLTM_H
