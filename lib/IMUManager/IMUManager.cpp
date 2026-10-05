#include "IMUManager.h"

IMUManager::IMUManager()
  : yawDeg_(0.0f), pitchDeg_(0.0f), rollDeg_(0.0f),
    yawRateDegPerSec_(0.0f), gyroBiasZ_(0.0f), lastUpdateUs_(0), initialized_(false) {
}

void IMUManager::begin() {
  initialized_ = mpu_.begin(MPU6050_I2C_ADDRESS);
  if (!initialized_) {
    LOG_ERROR("MPU6050 not found");
    return;
  }

  mpu_.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu_.setGyroRange(MPU6050_RANGE_250_DEG); 
  mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
  lastUpdateUs_ = micros();
}

void IMUManager::calibrate() {
  if (!initialized_) return;
  const uint16_t samples = 100;
  float sumZ = 0.0f;
  for (uint16_t i = 0; i < samples; ++i) {
    mpu_.getEvent(&accel_, &gyro_, &temp_);
    sumZ += gyro_.gyro.z;
    delay(5);
  }
  gyroBiasZ_ = sumZ / samples;
  yawRateDegPerSec_ = 0.0f;
  lastUpdateUs_ = micros();
}

void IMUManager::update() {
  if (!initialized_) return;
  mpu_.getEvent(&accel_, &gyro_, &temp_);
  const uint32_t nowUs = micros();
  const float dt = (nowUs - lastUpdateUs_) * 0.000001f;
  lastUpdateUs_ = nowUs;
  if (dt <= 0.0f || dt > 0.25f) return;

  const float radToDeg = 57.2957795f;
  yawRateDegPerSec_ = (gyro_.gyro.z - gyroBiasZ_) * radToDeg;
  yawDeg_ += yawRateDegPerSec_ * dt;
  if (yawDeg_ >= 360.0f) yawDeg_ -= 360.0f;
  if (yawDeg_ < 0.0f) yawDeg_ += 360.0f;

  const float ax = accel_.acceleration.x;
  const float ay = accel_.acceleration.y;
  const float az = accel_.acceleration.z;
  const float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * radToDeg;
  const float accRoll = atan2f(ay, az) * radToDeg;
  pitchDeg_ = 0.98f * (pitchDeg_ + gyro_.gyro.y * radToDeg * dt) + 0.02f * accPitch;
  rollDeg_ = 0.98f * (rollDeg_ + gyro_.gyro.x * radToDeg * dt) + 0.02f * accRoll;
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

bool IMUManager::healthy() const {
  return initialized_;
}
