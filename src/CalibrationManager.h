#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include <Arduino.h>
#include "config.h"

class CalibrationManager {
public:
  void begin();
  void update();
  void runMotorDirectionCalibration();
  void runEncoderCalibration();
  void runIMUCalibration();
  void runColorCalibration();
  void runQTRCalibration();
  void runServoCalibration();

private:
  bool active_;
};

#endif
