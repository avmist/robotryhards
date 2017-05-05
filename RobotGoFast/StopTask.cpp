#include "StopTask.h"

StopTask::StopTask() {

}

StopTask::StopTask(Task * parent, String name) : Task(STOP, parent, name) {
  
}

int StopTask::update() {

  Stepper::disableAll();

  state = COMPLETE;

  return CONTINUE;
  
}

void StopTask::init() {

  Task::init();

}
