#include "CalibrationManager.h"

void CalibrationManager::begin() {
  active_ = false;
  calibratingLine_ = false;
  lastReportMs_ = 0;
  motors_ = nullptr;
  encoders_ = nullptr;
  imu_ = nullptr;
  color_ = nullptr;
  line_ = nullptr;
  gripper_ = nullptr;
}

void CalibrationManager::configure(MotorController* motors, EncoderManager* encoders, IMUManager* imu,
                                   ColorSensor* color, LineSensor* line, GripperController* gripper) {
  motors_ = motors;
  encoders_ = encoders;
  imu_ = imu;
  color_ = color;
  line_ = line;
  gripper_ = gripper;
}

void CalibrationManager::update() {
  if (calibratingLine_ && line_ != nullptr) line_->calibrateSample();
  if (!DEBUG_ENABLED || COMPETITION_MODE || !Serial.available()) return;

  const char command = static_cast<char>(Serial.read());
  switch (command) {
    case 'm': runMotorDirectionCalibration(); break;
    case 'e': runEncoderCalibration(); break;
    case 'i': runIMUCalibration(); break;
    case 'c': runColorCalibration(); break;
    case 'q': runQTRCalibration(); break;
    case 's': runServoCalibration(); break;
    case 'x':
      calibratingLine_ = false;
      active_ = false;
      Serial.println(F("Calibration sampling stopped"));
      break;
    default: break;
  }
}

void CalibrationManager::runMotorDirectionCalibration() {
  active_ = true;
  Serial.println(F("Motor calibration: lift the robot, then test one wheel at a time."));
  Serial.println(F("Set MOTOR1_INVERTED..MOTOR4_INVERTED in src/config.h."));
  if (motors_ != nullptr) motors_->stopAll();
}

void CalibrationManager::runEncoderCalibration() {
  active_ = true;
  if (encoders_ == nullptr) return;
  Serial.println(F("Rotate each wheel exactly one output revolution and note signed ticks:"));
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(F("M")); Serial.print(i + 1); Serial.print(F("="));
    Serial.println(encoders_->getTicks(i));
  }
  Serial.println(F("Set the measured values in ENCODER_TICKS_PER_OUTPUT_REV_M1..M4."));
}

void CalibrationManager::runIMUCalibration() {
  if (imu_ == nullptr || !imu_->healthy()) return;
  Serial.println(F("Keep robot still while gyro bias is measured."));
  imu_->calibrate();
  Serial.println(F("IMU gyro bias calibration complete."));
}

void CalibrationManager::runColorCalibration() {
  if (color_ == nullptr) return;
  const robot::ColorReading& reading = color_->latestReading;
  Serial.print(F("R=")); Serial.print(reading.r);
  Serial.print(F(" G=")); Serial.print(reading.g);
  Serial.print(F(" B=")); Serial.print(reading.b);
  Serial.print(F(" C=")); Serial.println(reading.c);
  Serial.println(F("Record samples for each tile color and tune ColorSensor thresholds."));
}

void CalibrationManager::runQTRCalibration() {
  if (line_ == nullptr) return;
  if (!calibratingLine_) {
    line_->resetCalibration();
    calibratingLine_ = true;
    Serial.println(F("Move QTR across green floor and white lines; press x to stop."));
  } else {
    for (uint8_t i = 0; i < 8; ++i) {
      Serial.print(line_->sensorValues[i]);
      Serial.print(i == 7 ? '\n' : '\t');
    }
  }
}

void CalibrationManager::runServoCalibration() {
  if (gripper_ != nullptr) gripper_->calibrateGripper();
}
