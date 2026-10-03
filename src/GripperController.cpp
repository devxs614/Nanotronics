#include "GripperController.h"

void GripperController::begin() {
  servo_.attach(GRIPPER_SERVO_PIN);
  angle_ = GRIPPER_OPEN_ANGLE;
  servo_.write(angle_);
}

void GripperController::update() {
  servo_.write(angle_);
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
}
