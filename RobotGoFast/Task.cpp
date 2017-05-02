#include "Task.h"

Task::Task(Type type, Task * parent, String name) : name(name), type(type) {

  numParents = 0;
  numChildren = 0;
  traversed = false; 
  
  if(parent) {
    
    // Add to parents list
    addParent(parent);

    // Add to parents children list
    parent->addChild(this);
    
  }
  
}

Task::Task(Type type, Task * mom, Task * dad, String name) : name(name), type(type) {

  numParents = 0;
  numChildren = 0;
  traversed = false;

  // Add to parents list
  addParent(mom);
  addParent(dad);

  // Add to children list
  mom->addChild(this);
  dad->addChild(this);
  
}

void Task::addChild(Task * task) {
  children[numChildren++] = task;
}

void Task::addParent(Task * task) {
  parents[numParents++] = task;
}

bool Task::update() {
  SerialUSB.print("Task Task\n");
  state = ERROR;
  return false;
}

void Task::init() {
  SerialUSB.print("\nTask ");
  SerialUSB.print(name);
  SerialUSB.print(" started.\n");
}

Task * Task::getUntreversedChild() {

  for(int i = 0; i < numChildren; ++i) {

    if(children[i]->traversed == false) { 
      /*SerialUSB.print("Transitioning to ");
      SerialUSB.print(children[i]->type);
      SerialUSB.print("\n");*/
      return children[i];
    }
    
  }

  state = ERROR;
  Stepper::disableAll();

  SerialUSB.print("Transitioning to NULL");
  return NULL;
  
}
