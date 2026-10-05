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
  void calibrateGripper();
  void testGripper();
  int angle() const;

private:
  Servo servo_;
  int angle_;
  uint8_t testStep_;
  uint32_t testStepStartedMs_;
  bool testActive_;
};

#endif
