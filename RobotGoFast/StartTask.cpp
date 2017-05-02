#include "StartTask.h"

StartTask::StartTask(Task * parent, String name) : Task(START, parent, name) {
  
}

StartTask::StartTask(Task * mom, Task * dad, String name) : Task(START, mom, dad, name) {
  
}

bool StartTask::update() {
  Stepper::enableAll();
  state = RUNNING;
  traversed = true;
  return true;
}

void StartTask::init() {
  Task::init();
}
