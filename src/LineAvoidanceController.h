#ifndef LINE_AVOIDANCE_CONTROLLER_H
#define LINE_AVOIDANCE_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

class LineAvoidanceController {
public:
  void begin();
  void update();
  bool whiteLineDetected() const;

private:
  bool whiteLineDetected_;
};

#endif
