#include "DelayTask.h"

DelayTask::DelayTask(Task * parent, unsigned long delay, String name) : Task(DELAY, parent, name) {
  this->delay = delay;
}

DelayTask::DelayTask(Task * mom, Task * dad, unsigned long delay, String name) : Task(DELAY, mom, dad, name) {
  this->delay = delay;
}

bool DelayTask::update() {

  unsigned long dt = micros() - startTime;
  
  return (dt > delay);

}

void DelayTask::init() {

  Task::init();
  
  startTime = micros();
  
}
