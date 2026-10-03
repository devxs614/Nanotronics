#include "MecanumDrive.h"

void MecanumDrive::begin(MotorController* motorController) {
  motorController_ = motorController;
  wheelSigns_[0] = 1.0f;
  wheelSigns_[1] = 1.0f;
  wheelSigns_[2] = 1.0f;
  wheelSigns_[3] = 1.0f;
}

void MecanumDrive::inverseKinematics(float vx, float vy, float omega, float wheelSpeeds[4]) {
  // Base mecanum mapping in robot-relative frame.
  // These signs are centralized and can be calibrated in config.
  wheelSpeeds[0] = (vx + vy + omega) * wheelSigns_[0];
  wheelSpeeds[1] = (-vx + vy + omega) * wheelSigns_[1];
  wheelSpeeds[2] = (-vx + vy - omega) * wheelSigns_[2];
  wheelSpeeds[3] = (vx + vy - omega) * wheelSigns_[3];
}

void MecanumDrive::forwardKinematics(float wheelSpeeds[4], float& vx, float& vy, float& omega) {
  vx = (wheelSpeeds[0] + wheelSpeeds[3] - wheelSpeeds[1] - wheelSpeeds[2]) / 4.0f;
  vy = (wheelSpeeds[0] + wheelSpeeds[1] + wheelSpeeds[2] + wheelSpeeds[3]) / 4.0f;
  omega = (wheelSpeeds[0] - wheelSpeeds[1] + wheelSpeeds[2] - wheelSpeeds[3]) / 4.0f;
}

void MecanumDrive::drive(float vx, float vy, float omega) {
  float wheelSpeeds[4];
  inverseKinematics(vx, vy, omega, wheelSpeeds);
  for (uint8_t i = 0; i < 4; ++i) {
    motorController_->setMotorPWM(i, (int16_t)constrain(wheelSpeeds[i], -255.0f, 255.0f));
  }
}

void MecanumDrive::driveForward(float speed) {
  drive(0.0f, speed, 0.0f);
}

void MecanumDrive::driveBackward(float speed) {
  drive(0.0f, -speed, 0.0f);
}

void MecanumDrive::strafeLeft(float speed) {
  drive(-speed, 0.0f, 0.0f);
}

void MecanumDrive::strafeRight(float speed) {
  drive(speed, 0.0f, 0.0f);
}

void MecanumDrive::rotateCW(float speed) {
  drive(0.0f, 0.0f, -speed);
}

void MecanumDrive::rotateCCW(float speed) {
  drive(0.0f, 0.0f, speed);
}

void MecanumDrive::stop() {
  for (uint8_t i = 0; i < 4; ++i) {
    motorController_->setMotorPWM(i, 0);
  }
}
