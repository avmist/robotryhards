#ifndef TASK_H
#define TASK_H

#include <Arduino.h>
#include "RobotState.h"
#include "Stepper.h"

extern RobotState state;

class Task {

public:

  enum Return { CONTINUE = 0, END = 1, ERROR = -1 };

  enum Type { START = 1, STOP = 0, TURN = 2, DELAY = 3, GO2 = 5 };
  const String name;
  const Type type;

protected:

  Task * parent;
  Task * children[3];
  int numChildren;
  bool traversed;
  
public:

  // Constructors
  Task(Type type, Task * parent, String name);
  Task();

  // Methods
  virtual int update();
  virtual void init();
  Task * getUntreversedChild();
  Task * getParent();
  void setParent(Task * task);
  void print(int depth);
  void addChild(Task * task);

private:


  
};

#endif // TASK_H
