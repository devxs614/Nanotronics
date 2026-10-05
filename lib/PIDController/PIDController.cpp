#include "PIDController.h"

PIDController::PIDController()
  : kp_(0.0f), ki_(0.0f), kd_(0.0f),
    setpoint_(0.0f), lastMeasurement_(0.0f), integral_(0.0f),
    output_(0.0f), lastDerivative_(0.0f),
    minOutput_(-255.0f), maxOutput_(255.0f),
    lastUpdateMs_(millis()), sampleTimeMs_(20), integralLimit_(1000.0f),
    deadband_(0.0f), enabled_(true) {
}

void PIDController::setTunings(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setSampleTimeMs(uint16_t ms) {
  if (ms > 0) sampleTimeMs_ = ms;
}

void PIDController::setOutputLimits(float minOut, float maxOut) {
  if (minOut >= maxOut) return;
  minOutput_ = minOut;
  maxOutput_ = maxOut;
  output_ = constrain(output_, minOutput_, maxOutput_);
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
  lastDerivative_ = 0.0f;
  lastMeasurement_ = 0.0f;
  lastUpdateMs_ = millis();
}

void PIDController::enable(bool enabled) {
  enabled_ = enabled;
}

float PIDController::update(float measurement) {
  if (!enabled_) return 0.0f;

  const uint32_t nowMs = millis();
  const uint32_t elapsedMs = nowMs - lastUpdateMs_;
  if (elapsedMs < sampleTimeMs_) return output_;

  const float dt = elapsedMs * 0.001f;
  float error = setpoint_ - measurement;
  if (fabs(error) < deadband_) error = 0.0f;

  const float previousOutput = output_;
  const float candidateIntegral = constrain(integral_ + error * dt, -integralLimit_, integralLimit_);
  const float rawDerivative = -(measurement - lastMeasurement_) / dt;
  const float derivative = 0.25f * rawDerivative + 0.75f * lastDerivative_;
  const float adjustment = (kp_ * error) + (ki_ * candidateIntegral) + (kd_ * derivative);

  output_ = constrain(adjustment, minOutput_, maxOutput_);
  const bool saturatedHigh = adjustment > maxOutput_ && error > 0.0f;
  const bool saturatedLow = adjustment < minOutput_ && error < 0.0f;
  if (!saturatedHigh && !saturatedLow) integral_ = candidateIntegral;
  lastMeasurement_ = measurement;
  lastDerivative_ = derivative;
  lastUpdateMs_ = nowMs;
  if (error == 0.0f && previousOutput == 0.0f) integral_ = 0.0f;
  return output_;
}

float PIDController::getOutput() const {
  return output_;
}
