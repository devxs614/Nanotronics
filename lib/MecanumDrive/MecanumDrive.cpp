#include "MecanumDrive.h"
#include "config.h"

void MecanumDrive::begin(MotorController* motorController) {
  begin(motorController, nullptr);
}

void MecanumDrive::begin(MotorController* motorController, EncoderManager* encoderManager) {
  motorController_ = motorController;
  encoderManager_ = encoderManager;
  wheelSigns_[0] = 1.0f;
  wheelSigns_[1] = 1.0f;
  wheelSigns_[2] = 1.0f;
  wheelSigns_[3] = 1.0f;

  const float kp[4] = {KP_M1, KP_M2, KP_M3, KP_M4};
  const float ki[4] = {KI_M1, KI_M2, KI_M3, KI_M4};
  const float kd[4] = {KD_M1, KD_M2, KD_M3, KD_M4};

  for (uint8_t i = 0; i < 4; ++i) {
    targetWheelSpeed_[i] = 0.0f;
    wheelPid_[i].setTunings(kp[i], ki[i], kd[i]);
    wheelPid_[i].setSampleTimeMs(20);
    wheelPid_[i].setOutputLimits(-MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  }
  lastPidUpdateMs_ = millis();
}

void MecanumDrive::inverseKinematics(float vx, float vy, float omega, float wheelSpeeds[4]) {
  const float rotationalSpeed = omega * ((WHEEL_BASE_MM + TRACK_WIDTH_MM) * 0.5f);
  // Orden de motores: 0: FL, 1: FR, 2: RL, 3: RR
  wheelSpeeds[0] = (vy - vx + rotationalSpeed) * wheelSigns_[0];
  wheelSpeeds[1] = (vy + vx + rotationalSpeed) * wheelSigns_[1];
  wheelSpeeds[2] = (vy + vx - rotationalSpeed) * wheelSigns_[2];
  wheelSpeeds[3] = (vy - vx - rotationalSpeed) * wheelSigns_[3];
}

void MecanumDrive::forwardKinematics(float wheelSpeeds[4], float& vx, float& vy, float& omega) {
  vx = (-wheelSpeeds[0] + wheelSpeeds[1] + wheelSpeeds[2] - wheelSpeeds[3]) / 4.0f;
  vy = (wheelSpeeds[0] + wheelSpeeds[1] + wheelSpeeds[2] + wheelSpeeds[3]) / 4.0f;
  const float radius = (WHEEL_BASE_MM + TRACK_WIDTH_MM) * 0.5f;
  omega = radius > 0.0f
    ? (wheelSpeeds[0] + wheelSpeeds[1] - wheelSpeeds[2] - wheelSpeeds[3]) / (4.0f * radius)
    : 0.0f;
}

void MecanumDrive::drive(float vx, float vy, float omega) {
  if (motorController_ == nullptr) return;

  inverseKinematics(vx, vy, omega, targetWheelSpeed_);

  float maxMagnitude = 0.0f;
  for (uint8_t i = 0; i < 4; ++i) {
    const float magnitude = fabs(targetWheelSpeed_[i]);
    if (magnitude > maxMagnitude) maxMagnitude = magnitude;
  }

  const float maxSpeed = (RPM_TARGET_MAX * PI * WHEEL_DIAMETER_MM) / 60.0f;
  if (maxMagnitude > maxSpeed && maxMagnitude > 0.0f) {
    const float scale = maxSpeed / maxMagnitude;
    for (uint8_t i = 0; i < 4; ++i) targetWheelSpeed_[i] *= scale;
  }

  for (uint8_t i = 0; i < 4; ++i) {
    float targetRpm = (targetWheelSpeed_[i] / (PI * WHEEL_DIAMETER_MM)) * 60.0f;
    wheelPid_[i].setSetpoint(targetRpm);
  }
}

void MecanumDrive::update() {
  if (motorController_ == nullptr) return;

  const uint32_t now = millis();
  if (now - lastPidUpdateMs_ < 20) return;
  lastPidUpdateMs_ = now;

  const float maxSpeed = (RPM_TARGET_MAX * PI * WHEEL_DIAMETER_MM) / 60.0f;

  for (uint8_t i = 0; i < 4; ++i) {
    // Si la velocidad objetivo es cero, detener el motor y reiniciar PID
    if (fabs(targetWheelSpeed_[i]) < 0.1f) {
      wheelPid_[i].reset();
      motorController_->setMotorPWM(i, 0);
      continue;
    }

    // 1. Calculo PWM Base de Alimentación Directa (Feedforward)
    float feedforwardPwm = (targetWheelSpeed_[i] / maxSpeed) * MOTOR_PWM_MAX;

    // 2. Ajuste Fino PID por Encoders
    float pidTrim = 0.0f;
    if (encoderManager_ != nullptr) {
      pidTrim = wheelPid_[i].update(encoderManager_->getRPM(i));
    }

    float totalPwm = feedforwardPwm + pidTrim;

    // 3. Garantizar el Umbral Mínimo de PWM para vencer la fricción estática
    if (fabs(totalPwm) > 0.1f && fabs(totalPwm) < MOTOR_PWM_MIN) {
      totalPwm = (totalPwm > 0) ? MOTOR_PWM_MIN : -MOTOR_PWM_MIN;
    }

    motorController_->setMotorPWM(i, static_cast<int16_t>(constrain(totalPwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX)));
  }
}

void MecanumDrive::driveForward(float speed) { drive(0.0f, speed, 0.0f); }
void MecanumDrive::driveBackward(float speed) { drive(0.0f, -speed, 0.0f); }
void MecanumDrive::strafeLeft(float speed) { drive(-speed, 0.0f, 0.0f); }
void MecanumDrive::strafeRight(float speed) { drive(speed, 0.0f, 0.0f); }
void MecanumDrive::rotateCW(float speed) { drive(0.0f, 0.0f, -speed); }
void MecanumDrive::rotateCCW(float speed) { drive(0.0f, 0.0f, speed); }

void MecanumDrive::stop() {
  if (motorController_ == nullptr) return;
  for (uint8_t i = 0; i < 4; ++i) {
    targetWheelSpeed_[i] = 0.0f;
    wheelPid_[i].setSetpoint(0.0f);
    wheelPid_[i].reset();
    motorController_->setMotorPWM(i, 0);
  }
  motorController_->stopAll();
}