#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include "Arduino.h"

enum RobotState { IDLE, RUNNING, ERROR, COMPLETE };

const float IR_SPACING = 4.6625;

void printDouble(float val, unsigned int precision);

#endif // ROBOTSTATE_H
