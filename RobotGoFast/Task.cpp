#include "Task.h"

Task::Task(Type type, Task * parent, String name) : name(name), type(type) {

  numParents = 0;
  numChildren = 0;
  traversed = false; 
  
  if(parent) {
    
    // Add to parents list
    setParent(parent);

    // Add to parents children list
    parent->addChild(this);
    
  }
  
}

void Task::addChild(Task * task) {
  children[numChildren++] = task;
}

void Task::setParent(Task * task) {
  parent = task;
}

Task * Task::getParent() {
  return this->parent;
}

bool Task::update() {
  SerialUSB.print("Task Task\n");
  return Task::ERROR;
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

  Stepper::disableAll();

  SerialUSB.print("Transitioning to NULL");
  
  return NULL;
  
}
