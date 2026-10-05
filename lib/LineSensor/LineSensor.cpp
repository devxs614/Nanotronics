#include "LineSensor.h"

void LineSensor::begin() {
  initialized_ = true;
  calibrationStarted_ = false;
  for (uint8_t i = 0; i < 8; ++i) {
    sensorValues[i] = 0;
    lineCalibrationMin[i] = 0;
    lineCalibrationMax[i] = 1023;
  }
  const uint8_t pins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
  qtr_.setTypeAnalog();
  qtr_.setSensorPins(pins, 8);
  lastObservation_ = {false, 0.0f, 0, 0.0f, false, false, false};
}

void LineSensor::update() {
  if (!initialized_) return;
  qtr_.read(sensorValues);
  detectWhiteLine();
}

bool LineSensor::detectWhiteLine() {
  uint8_t active = 0;
  uint8_t left = 0, center = 0, right = 0;
  uint32_t weightedPosition = 0;
  uint32_t totalSignal = 0;

  for (uint8_t i = 0; i < 8; ++i) {
    const uint16_t span = lineCalibrationMax[i] > lineCalibrationMin[i]
      ? lineCalibrationMax[i] - lineCalibrationMin[i] : 1;
    const int32_t calibrated =
      (static_cast<int32_t>(sensorValues[i]) - lineCalibrationMin[i]) * 1000L / span;
    const uint16_t strength = static_cast<uint16_t>(constrain(calibrated, 0, 1000));
    if (strength >= WHITE_LINE_ACTIVE_THRESHOLD) {
      ++active;
      weightedPosition += static_cast<uint32_t>(strength) * (i * 1000U);
      totalSignal += strength;
      if (i < 3) ++left;
      else if (i < 5) ++center;
      else ++right;
    }
  }

  lastObservation_.detected = active > 0;
  lastObservation_.activeSensors = active;
  lastObservation_.normalizedPosition = totalSignal
    ? (static_cast<float>(weightedPosition) / (totalSignal * 7000.0f)) * 2.0f - 1.0f
    : 0.0f;
  lastObservation_.confidence = active ? min(1.0f, active / 3.0f) : 0.0f;
  lastObservation_.leftBlocked = left > 0;
  lastObservation_.centerBlocked = center > 0;
  lastObservation_.rightBlocked = right > 0;
  return lastObservation_.detected;
}

bool LineSensor::hasLeftLine() const {
  return lastObservation_.leftBlocked;
}

bool LineSensor::hasCenterLine() const {
  return lastObservation_.centerBlocked;
}

bool LineSensor::hasRightLine() const {
  return lastObservation_.rightBlocked;
}

const robot::LineObservation& LineSensor::observation() const {
  return lastObservation_;
}

void LineSensor::calibrateSample() {
  if (!initialized_) return;
  qtr_.read(sensorValues);
  if (!calibrationStarted_) {
    for (uint8_t i = 0; i < 8; ++i) {
      lineCalibrationMin[i] = sensorValues[i];
      lineCalibrationMax[i] = sensorValues[i];
    }
    calibrationStarted_ = true;
    return;
  }
  for (uint8_t i = 0; i < 8; ++i) {
    if (sensorValues[i] < lineCalibrationMin[i]) lineCalibrationMin[i] = sensorValues[i];
    if (sensorValues[i] > lineCalibrationMax[i]) lineCalibrationMax[i] = sensorValues[i];
  }
}

void LineSensor::resetCalibration() {
  calibrationStarted_ = false;
  for (uint8_t i = 0; i < 8; ++i) {
    lineCalibrationMin[i] = 0;
    lineCalibrationMax[i] = 1023;
  }
}
