#ifndef LINE_AVOIDANCE_CONTROLLER_H
#define LINE_AVOIDANCE_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include "LineSensor.h"
#include "MecanumDrive.h"

class LineAvoidanceController {
public:
  void begin();
  void configure(LineSensor* sensor, MecanumDrive* drive);
  void update();
  bool whiteLineDetected() const;
  bool corridorClear() const;

private:
  LineSensor* sensor_;
  MecanumDrive* drive_;
  bool whiteLineDetected_;
  bool corridorClear_;
};

#endif
