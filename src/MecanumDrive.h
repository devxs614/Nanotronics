#ifndef MECANUM_DRIVE_H
#define MECANUM_DRIVE_H

#include <Arduino.h>
#include "config.h"
#include "MotorController.h"

class MecanumDrive {
public:
  void begin(MotorController* motorController);
  void drive(float vx, float vy, float omega);
  void driveForward(float speed);
  void driveBackward(float speed);
  void strafeLeft(float speed);
  void strafeRight(float speed);
  void rotateCW(float speed);
  void rotateCCW(float speed);
  void stop();

  void inverseKinematics(float vx, float vy, float omega, float wheelSpeeds[4]);
  void forwardKinematics(float wheelSpeeds[4], float& vx, float& vy, float& omega);

private:
  MotorController* motorController_;
  float wheelSigns_[4];
};

#endif
