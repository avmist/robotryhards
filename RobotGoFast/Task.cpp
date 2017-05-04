#include "Task.h"

Task::Task(Type type, Task * parent, String name) : name(name), type(type) {

  numChildren = 0;
  traversed = false; 
  
  if(parent) {
    
    // Add to parents list
    this->parent = parent;

    // Add to parents children list
    parent->addChild(this);
    
  }
  
}

void Task::print(int depth) {

  for(int i = 0; i < depth; ++i) {
    SerialUSB.print("- ");
  }

  SerialUSB.println(name);

  for(int i = 0; i < numChildren; ++i) {
    children[i]->print(depth + 1);
  }

}

void Task::addChild(Task * task) {
  children[numChildren++] = task;
}

Task * Task::getParent() {
  return this->parent;
}

int Task::update() {
  SerialUSB.print("Task Task\n");
  return Task::ERROR;
}

void Task::init() {
  SerialUSB.print("\nTask ");
  SerialUSB.print(name);
  SerialUSB.print(" started.\n");
}

Task * Task::getUntreversedChild() {

  SerialUSB.print("Task ");
  SerialUSB.print(this->name);
  SerialUSB.print(" has ");
  SerialUSB.print(numChildren);
  SerialUSB.println(" children.");

  for(int i = 0; i < numChildren; ++i) {

    if(children[i]->traversed == false) {

      // SerialUSB.print("Transitioning to ");
      // SerialUSB.print(children[i]->name);
      // SerialUSB.print("\n");

      return children[i];

    } else {

      SerialUSB.print("Task ");
      SerialUSB.print(children[i]->name);
      SerialUSB.println(" traversed.");

    }
    
  }

  //Stepper::disableAll();

  return NULL;
  
}
