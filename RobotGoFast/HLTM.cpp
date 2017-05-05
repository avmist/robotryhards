#include "HLTM.h"

HLTM::HLTM(Task * rootTask) {
  this->currentTask = rootTask;
}

void HLTM::init() {
  this->currentTask->init();
}

void HLTM::initCurrentTask() {

  if(!currentTask) {

    state = ERROR;

    while(true) {
      SerialUSB.print("Task is NULL\n");
      delay(100);
    }
    
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
    
  } else if(currentTask->type == Task::GO2) {

    GoTask2 * goTask2 = static_cast<GoTask2*>(currentTask);

    goTask2->init();
    
  } else {

    state = ERROR;

    while(true) {

      SerialUSB.print("Task is ");
      SerialUSB.print(currentTask->type);
      SerialUSB.print("\n");
      delay(100);

    }
    
  }

}

int HLTM::updateCurrentTask() {

  //SerialUSB.println("Update");

  if(!currentTask) {

    state = ERROR;

    while(true) {
      SerialUSB.print("Task is NULL\n");
      delay(100);
    }

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
    
  } else if(currentTask->type == Task::GO2) {

    GoTask2 * goTask2 = static_cast<GoTask2*>(currentTask);

    return goTask2->update();
    
  } else {
    
    state = ERROR;

    while(true) {

      SerialUSB.print("Task is ");
      SerialUSB.print(currentTask->type);
      SerialUSB.print("\n");
      delay(100);

    }

    return Task::ERROR;
    
  }

}

void HLTM::update() {

  int taskStatus = updateCurrentTask();

  if(taskStatus == Task::ERROR) {
    
    state = ERROR;

    // Busy wait
    while(true) {
      SerialUSB.print("Task returned error.\n");
      delay(100);
    }

  } else if(taskStatus == Task::END) {

    SerialUSB.print("Task ");
    SerialUSB.print(currentTask->name);
    SerialUSB.println(" ended.");
    SerialUSB.println();

    // Are we backtracking?
    if(backtracking == NOT_BACKTRACKING) {

      SerialUSB.println("Not backtracking, advancing to next task...");

      // No

      // Do next task
      currentTask = currentTask->getUntreversedChild();

      initCurrentTask();

    } else {

      // Yes

      // Is there a branch here?
      Task * previousTask = currentTask->getParent();

      if(previousTask == NULL) {

        state = ERROR;

        // Busy wait
        while(true) {
          SerialUSB.println("Error: Parent is NULL");
          delay(100);
        }

      }

      Task * previousTaskChild = previousTask->getUntreversedChild();

      if(previousTaskChild) {

        // Branch exists, go there

        SerialUSB.print("Branching to ");
        SerialUSB.println(previousTaskChild->name);

        backtracking = NOT_BACKTRACKING;

        currentTask = previousTaskChild;

        initCurrentTask();

      } else {

        // No, go back

        SerialUSB.print("Reverting to ");
        SerialUSB.println(previousTask->name);

        currentTask = previousTask;

        initCurrentTask();

      }

    }

  }
  
}
