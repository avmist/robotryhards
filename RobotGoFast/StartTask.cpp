#include "StartTask.h"

StartTask::StartTask() {
	
}

StartTask::StartTask(Task * parent, String name) : Task(START, parent, name) { }

int StartTask::update() {

  Stepper::enableAll();

  state = RUNNING;

  traversed = true;

  return END;
  
}

void StartTask::init() {

  Task::init();

}
