#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include <Arduino.h>
#include "config.h"
#include "MotorController.h"
#include "EncoderManager.h"
#include "IMUManager.h"
#include "ColorSensor.h"
#include "LineSensor.h"
#include "GripperController.h"

class CalibrationManager {
public:
  void begin();
  void configure(MotorController* motors, EncoderManager* encoders, IMUManager* imu,
                 ColorSensor* color, LineSensor* line, GripperController* gripper);
  void update();
  void runMotorDirectionCalibration();
  void runEncoderCalibration();
  void runIMUCalibration();
  void runColorCalibration();
  void runQTRCalibration();
  void runServoCalibration();

private:
  MotorController* motors_;
  EncoderManager* encoders_;
  IMUManager* imu_;
  ColorSensor* color_;
  LineSensor* line_;
  GripperController* gripper_;
  bool active_;
  bool calibratingLine_;
  uint32_t lastReportMs_;
};

#endif
