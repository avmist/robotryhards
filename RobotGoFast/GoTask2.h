#ifndef GoTask2_H
#define GoTask2_H

#include "Task.h"
#include "LinearFit.h"
#include "RobotState.h"
#include "LED.h"
#include "Vec2.h"
#include "PID.h"
#include "Adafruit_VL6180X.h"

extern Stepper leftMotor;
extern Stepper rightMotor;

extern LED led;

extern LinearFit ir0;
extern LinearFit ir1;
extern LinearFit ir2;
extern LinearFit ir3;

extern Adafruit_VL6180X vl;

extern Backtracking backtracking;

class GoTask2 : public Task {

public:

  enum State { BOTH_WALLS, LEFT_WALL, RIGHT_WALL, NO_WALL };

private:

  int distance;
  PID distPD;
  PID angPD;
  unsigned long stepsToTravel, stepsTraveled;
  unsigned long lastStatusPing;
  State wallState;

public:

  // Constructors
  GoTask2(Task * parent, float distance, String name);
  // Methods
  int update() override;
  void init() override;
  void initPID();
  
};

#endif // GoTask2_H