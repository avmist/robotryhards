#include "HLTM.h"

HLTM::HLTM(Task * rootTask) {
  this->currentTask = rootTask;
}

void HLTM::init() {
  this->currentTask->init();
}

void HLTM::initCurrentTask() {

  if(!currentTask) {

    SerialUSB.print("Task is NULL\n");
    state = ERROR;
    
  } else if(currentTask->type == Task::START) {

    StartTask * startTask = static_cast<StartTask*>(currentTask);

    startTask->init();
    
  } else if(currentTask->type == Task::STOP) {

    StopTask * stopTask = static_cast<StopTask*>(currentTask);

    stopTask->init();
    
  } else if(currentTask->type == Task::TURN) {

    TurnTask * turnTask = static_cast<TurnTask*>(currentTask);
    
    turnTask->init();
    
  } else if(currentTask->type == Task::DELAY) {

    DelayTask * delayTask = static_cast<DelayTask*>(currentTask);
    
    delayTask->init();
    
  } else if(currentTask->type == Task::GO) {

    GoTask * goTask = static_cast<GoTask*>(currentTask);

    goTask->init();
    
  } else if(currentTask->type == Task::GO2) {

    GoTask2 * goTask2 = static_cast<GoTask2*>(currentTask);

    goTask2->init();
    
  } else {
    
    SerialUSB.print("Task is ");
    SerialUSB.print(currentTask->type);
    SerialUSB.print("\n");
    state = ERROR;
    
  }

}

int HLTM::updateCurrentTask() {

  if(!currentTask) {

    SerialUSB.print("Task is NULL\n");
    state = ERROR;
    return Task::ERROR;
    
  } else if(currentTask->type == Task::START) {

    StartTask * startTask = static_cast<StartTask*>(currentTask);

    return startTask->update();
    
  } else if(currentTask->type == Task::STOP) {

    StopTask * stopTask = static_cast<StopTask*>(currentTask);

    return stopTask->update();
    
  } else if(currentTask->type == Task::TURN) {

    TurnTask * turnTask = static_cast<TurnTask*>(currentTask);
    
    return turnTask->update();
    
  } else if(currentTask->type == Task::DELAY) {

    DelayTask * delayTask = static_cast<DelayTask*>(currentTask);
    
    return delayTask->update();
    
  } else if(currentTask->type == Task::GO) {

    GoTask * goTask = static_cast<GoTask*>(currentTask);

    return goTask->update();
    
  } else if(currentTask->type == Task::GO2) {

    GoTask2 * goTask2 = static_cast<GoTask2*>(currentTask);

    return goTask2->update();
    
  } else {
    
    SerialUSB.print("Task is ");
    SerialUSB.print(currentTask->type);
    SerialUSB.print("\n");
    state = ERROR;

    return Task::ERROR;
    
  }

}

void HLTM::update() {

  //SerialUSB.print("Boop\n");

  int status = updateCurrentTask();

  if(status == Task::ERROR) {
    
    SerialUSB.print("Task returned error.\n");
    
    // Busy wait
    while(true);

  }

  if(status == Task::END) {

    currentTask = currentTask->getUntreversedChild();

    initCurrentTask();

  } else if(status == Task::BACKTRACK) {

    currentTask = currentTask->getParent();

    currentTask = currentTask->getUntreversedChild();

    initCurrentTask();

  }
  
}
