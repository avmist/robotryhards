#ifndef HLTM_H
#define HLTM_H

#include "Task.h"
#include "TurnTask.h"
#include "DelayTask.h"
#include "StartTask.h"
#include "StopTask.h"
#include "GoTask.h"
#include "GoTask2.h"

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
  void initCurrentTask();
  int updateCurrentTask();

private:
  
};

#endif // HLTM_H
