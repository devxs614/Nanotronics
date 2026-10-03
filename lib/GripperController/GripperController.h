#ifndef GRIPPER_CONTROLLER_H
#define GRIPPER_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>
#include "config.h"

class GripperController {
public:
  void begin();
  void update();
  void open();
  void close();
  void hold();
  void setAngle(int angle);

private:
  Servo servo_;
  int angle_;
};

#endif
