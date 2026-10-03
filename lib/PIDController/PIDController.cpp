#include "PIDController.h"

PIDController::PIDController()
  : kp_(0.0f), ki_(0.0f), kd_(0.0f),
    setpoint_(0.0f), lastMeasurement_(0.0f), integral_(0.0f),
    output_(0.0f), minOutput_(-255.0f), maxOutput_(255.0f),
    lastUpdateMs_(millis()), sampleTimeMs_(20), enabled_(true) {
}

void PIDController::setTunings(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setSampleTimeMs(uint16_t ms) {
  sampleTimeMs_ = ms;
}

void PIDController::setOutputLimits(float minOut, float maxOut) {
  minOutput_ = minOut;
  maxOutput_ = maxOut;
}

void PIDController::setSetpoint(float setpoint) {
  setpoint_ = setpoint;
}

void PIDController::setMeasurement(float measurement) {
  lastMeasurement_ = measurement;
}

void PIDController::reset() {
  integral_ = 0.0f;
  output_ = 0.0f;
  lastMeasurement_ = 0.0f;
  lastUpdateMs_ = millis();
}

void PIDController::enable(bool enabled) {
  enabled_ = enabled;
}

float PIDController::update(float measurement) {
  if (!enabled_) return 0.0f;

  const uint32_t nowMs = millis();
  if ((nowMs - lastUpdateMs_) < sampleTimeMs_) return output_;

  const float error = setpoint_ - measurement;
  integral_ += error;

  const float derivative = measurement - lastMeasurement_;
  float adjustment = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);

  output_ = constrain(adjustment, minOutput_, maxOutput_);
  lastMeasurement_ = measurement;
  lastUpdateMs_ = nowMs;
  return output_;
}

float PIDController::getOutput() const {
  return output_;
}
