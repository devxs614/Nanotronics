#ifndef ROBOT_STATE_MACHINE_H
#define ROBOT_STATE_MACHINE_H

#include <Arduino.h>
#include "config.h"

class RobotStateMachine {
public:
  void begin();
  void update();
  robot::RobotMode getCurrentMode() const;
  void setMode(robot::RobotMode mode);

private:
  robot::RobotMode currentMode_;
};

#endif
