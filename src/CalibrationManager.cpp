#include "CalibrationManager.h"

void CalibrationManager::begin() {
  active_ = false;
}

void CalibrationManager::update() {
  // Calibration tasks are intentionally separate and can be invoked from diagnostics or a calibration menu.
}

void CalibrationManager::runMotorDirectionCalibration() {
  // Validate positive and negative wheel direction signs.
}

void CalibrationManager::runEncoderCalibration() {
  // Measure ticks per revolution and confirm inversion values.
}

void CalibrationManager::runIMUCalibration() {
  // Zero the IMU heading offset and validate drift.
}

void CalibrationManager::runColorCalibration() {
  // Place the TCS34725 over each target color and record the raw values.
}

void CalibrationManager::runQTRCalibration() {
  // Create the min/max white-line calibration arrays.
}

void CalibrationManager::runServoCalibration() {
  // Tune open, close, and hold servo positions.
}
