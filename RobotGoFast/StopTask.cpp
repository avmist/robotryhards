#include "StopTask.h"

StopTask::StopTask(Task * parent, String name) : Task(STOP, parent, name) {
  
}

StopTask::StopTask(Task * mom, Task * dad, String name) : Task(STOP, mom, dad, name) {

}

bool StopTask::update() {
  //Serial.print("Stop Task");
  Stepper::disableAll();
  state = COMPLETE;
  return false;
}

void StopTask::init() {
  Task::init();
}
