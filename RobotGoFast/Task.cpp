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
  Serial.print("Task Task\n");
  state = ERROR;
  return false;
}

void Task::init() {
  Serial.print("\nTask ");
  Serial.print(name);
  Serial.print(" started.\n");
}

Task * Task::getUntreversedChild() {

  for(int i = 0; i < numChildren; ++i) {

    if(children[i]->traversed == false) { 
      /*Serial.print("Transitioning to ");
      Serial.print(children[i]->type);
      Serial.print("\n");*/
      return children[i];
    }
    
  }

  state = ERROR;
  Stepper::disableAll();

  Serial.print("Transitioning to NULL");
  return NULL;
  
}
