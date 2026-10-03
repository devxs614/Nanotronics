#ifndef BALL_HANDLER_H
#define BALL_HANDLER_H

#include <Arduino.h>
#include "config.h"

class BallHandler {
public:
  void begin();
  void update();
  bool hasBall() const;
  void setBallState(robot::BallState state);

private:
  robot::BallState ballState_;
};

#endif
