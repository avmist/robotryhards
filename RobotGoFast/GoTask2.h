#ifndef GoTask2_H
#define GoTask2_H

#include "Task.h"
#include "LinearFit.h"
#include "RobotState.h"
#include "LED.h"
#include "Vec2.h"
#include "PID.h"

extern Stepper leftMotor;
extern Stepper rightMotor;

extern LED led;

extern LinearFit ir0;
extern LinearFit ir1;
extern LinearFit ir2;
extern LinearFit ir3;

class GoTask2 : public Task {

public:

	enum State { BOTH_WALLS, LEFT_WALL, RIGHT_WALL, NO_WALL };

private:

	int distance;
	PID distPD;
	PID angPD;
	unsigned long steps;
	unsigned long lastStatusPing;
	State state;

public:

  // Constructors
  GoTask2(Task * parent, float distance, String name);
  GoTask2(Task * mom, Task * dad, float distance, String name);

  // Methods
  bool update() override;
  void init() override;
  void initPID();
  
};

#endif // GoTask2_H
