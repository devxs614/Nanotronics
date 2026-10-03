#include "LineSensor.h"

void LineSensor::begin() {
  initialized_ = true;
  for (uint8_t i = 0; i < 8; ++i) {
    sensorValues[i] = 0;
    lineCalibrationMin[i] = 0;
    lineCalibrationMax[i] = 1023;
  }
  qtr_.setTypeRC();
  qtr_.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, 8);
}

void LineSensor::update() {
  if (!initialized_) return;
  qtr_.read(sensorValues);
}

bool LineSensor::detectWhiteLine() {
  uint8_t active = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    if (sensorValues[i] > 600) {
      ++active;
    }
  }
  lastObservation_.detected = active > 0;
  lastObservation_.activeSensors = active;
  return lastObservation_.detected;
}

bool LineSensor::hasLeftLine() const {
  return sensorValues[0] > 600 || sensorValues[1] > 600 || sensorValues[2] > 600;
}

bool LineSensor::hasCenterLine() const {
  return sensorValues[3] > 600 || sensorValues[4] > 600;
}

bool LineSensor::hasRightLine() const {
  return sensorValues[5] > 600 || sensorValues[6] > 600 || sensorValues[7] > 600;
}
