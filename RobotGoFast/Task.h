#ifndef TASK_H
#define TASK_H

#include <Arduino.h>
#include "RobotState.h"
#include "Stepper.h"

extern RobotState state;

class Task {

public:

  enum Return { CONTINUE = 0, END = 1, BACKTRACK = 2, ERROR = -1 };

  enum Type { START = 1, STOP = 0, TURN = 2, DELAY = 3, GO = 4, GO2 = 5 };
  const String name;
  const Type type;

protected:

  Task * parent;
  int numParents;
  Task * children[2];
  int numChildren;
  bool traversed;
  
public:

  // Constructors
  Task(Type type, Task * parent, String name);
  Task(Type type, Task * mom, Task * dad, String name);

  // Methods
  virtual bool update();
  virtual void init();
  Task * getUntreversedChild();
  Task * getParent();

private:

  // Methods
  void addChild(Task * task);
  void setParent(Task * task);
  
};

#endif // TASK_H
