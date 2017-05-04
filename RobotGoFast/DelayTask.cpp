#include "DelayTask.h"

DelayTask::DelayTask(Task * parent, unsigned long delay, String name) : Task(DELAY, parent, name) {
  this->delay = delay;
}

int DelayTask::update() {

  unsigned long dt = micros() - startTime;
  
  return (dt > delay);

}

void DelayTask::init() {

  Task::init();
  
  startTime = micros();
  
}
