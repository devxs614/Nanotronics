#include "IMUManager.h"

IMUManager::IMUManager()
  : yawDeg_(0.0f), pitchDeg_(0.0f), rollDeg_(0.0f), yawRateDegPerSec_(0.0f), yawOffset_(0.0f) {
}

void IMUManager::begin() {
  if (!mpu_.begin(MPU6050_I2C_ADDRESS)) {
    LOG_ERROR("MPU6050 not found");
    return;
  }

  mpu_.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu_.setGyroRange(MPU6050_RANGE_250_DEG); 
  mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void IMUManager::calibrate() {
  yawOffset_ = 0.0f;
  for (uint8_t i = 0; i < 20; ++i) {
    update();
    yawOffset_ += yawDeg_;
    delay(20);
  }
  yawOffset_ /= 20.0f;
}

void IMUManager::update() {
  mpu_.getEvent(&accel_, &gyro_, &temp_);
  float gyroZ = gyro_.gyro.z;
  yawRateDegPerSec_ = gyroZ * 57.2958f;

  yawDeg_ += yawRateDegPerSec_ * 0.02f;
  pitchDeg_ = accel_.acceleration.y * 57.2958f;
  rollDeg_ = accel_.acceleration.x * 57.2958f;
  yawDeg_ -= yawOffset_;
}

float IMUManager::getYaw() const {
  return yawDeg_;
}

float IMUManager::getPitch() const {
  return pitchDeg_;
}

float IMUManager::getRoll() const {
  return rollDeg_;
}

float IMUManager::getYawRate() const {
  return yawRateDegPerSec_;
}
