#include "GripperController.h"

void GripperController::begin() {
  servo_.attach(GRIPPER_SERVO_PIN);
  angle_ = GRIPPER_OPEN_ANGLE;
  testStep_ = 0;
  testStepStartedMs_ = millis();
  testActive_ = false;
  servo_.write(angle_);
}

void GripperController::update() {
  if (testActive_ && millis() - testStepStartedMs_ >= 800) {
    testStepStartedMs_ = millis();
    if (testStep_ == 0) {
      angle_ = GRIPPER_HOLD_ANGLE;
      testStep_ = 1;
    } else if (testStep_ == 1) {
      angle_ = GRIPPER_CLOSE_ANGLE;
      testStep_ = 2;
    } else {
      angle_ = GRIPPER_OPEN_ANGLE;
      testActive_ = false;
      testStep_ = 0;
    }
    servo_.write(angle_);
  }
}

void GripperController::open() {
  angle_ = GRIPPER_OPEN_ANGLE;
}

void GripperController::close() {
  angle_ = GRIPPER_CLOSE_ANGLE;
}

void GripperController::hold() {
  angle_ = GRIPPER_HOLD_ANGLE;
}

void GripperController::setAngle(int angle) {
  angle_ = constrain(angle, 0, 180);
  if (servo_.attached()) servo_.write(angle_);
}

void GripperController::calibrateGripper() {
  testGripper();
}

void GripperController::testGripper() {
  testActive_ = true;
  testStep_ = 0;
  testStepStartedMs_ = millis();
  open();
  if (servo_.attached()) servo_.write(angle_);
}

int GripperController::angle() const { return angle_; }
