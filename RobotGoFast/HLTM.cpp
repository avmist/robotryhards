#include "HLTM.h"

HLTM::HLTM(Task * rootTask) {
  this->currentTask = rootTask;
}

void HLTM::init() {
  this->currentTask->init();
}

void HLTM::update() {

  //SerialUSB.print("Boop\n");

  if(!currentTask) {
    
    SerialUSB.print("Task is NULL\n");
    state = ERROR;
    
  } else if(currentTask->type == Task::START) {

    StartTask * startTask = static_cast<StartTask*>(currentTask);
    if(startTask->update()) {
      currentTask = startTask->getUntreversedChild();
      this->currentTask->init();
    }
    
  } else if(currentTask->type == Task::STOP) {

    StopTask * stopTask = static_cast<StopTask*>(currentTask);
    if(stopTask->update()) {
      currentTask = stopTask->getUntreversedChild();
      this->currentTask->init();
    }
    
  } else if(currentTask->type == Task::TURN) {

    TurnTask * turnTask = static_cast<TurnTask*>(currentTask);
    if(turnTask->update()) {
      currentTask = turnTask->getUntreversedChild();
      this->currentTask->init();
    }
    
  } else if(currentTask->type == Task::DELAY) {

    DelayTask * delayTask = static_cast<DelayTask*>(currentTask);
    if(delayTask->update()) {
      currentTask = delayTask->getUntreversedChild();
      this->currentTask->init();
    }
    
  } else if(currentTask->type == Task::GO) {

    GoTask * goTask = static_cast<GoTask*>(currentTask);
    if(goTask->update()) {
      currentTask = goTask->getUntreversedChild();
      this->currentTask->init();
    }
    
  } else {
    
    SerialUSB.print("Task is ");
    SerialUSB.print(currentTask->type);
    SerialUSB.print("\n");
    state = ERROR;
    
  }
  
}
