#ifndef MAZE_NAVIGATOR_H
#define MAZE_NAVIGATOR_H

#include <Arduino.h>
#include "config.h"

class MazeNavigator {
public:
  void begin();
  void update();
  void reset();
  robot::PistaAState state() const;

private:
  robot::PistaAState currentState_;
};

#endif
