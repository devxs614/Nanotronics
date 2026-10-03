#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config.h"

class IMUManager {
public:
  IMUManager();
  void begin();
  void calibrate();
  void update();
  float getYaw() const;
  float getPitch() const;
  float getRoll() const;
  float getYawRate() const;

private:
  Adafruit_MPU6050 mpu_;
  sensors_event_t accel_;
  sensors_event_t gyro_;
  sensors_event_t temp_;
  float yawDeg_;
  float pitchDeg_;
  float rollDeg_;
  float yawRateDegPerSec_;
  float yawOffset_;
};

#endif
