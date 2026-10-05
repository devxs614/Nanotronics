#include "Odometry.h"

void Odometry::begin() {
  encoders_ = nullptr;
  imu_ = nullptr;
  for (uint8_t i = 0; i < 4; ++i) previousTicks_[i] = 0;
  x_ = 0.0f;
  y_ = 0.0f;
  theta_ = 0.0f;
  imuHeadingOffsetDeg_ = 0.0f;
  pose_ = {0.0f, 0.0f, 0.0f};
}

void Odometry::begin(EncoderManager* encoders, IMUManager* imu) {
  begin();
  encoders_ = encoders;
  imu_ = imu;
  if (encoders_ != nullptr) {
    for (uint8_t i = 0; i < 4; ++i) previousTicks_[i] = encoders_->getTicks(i);
  }
}

void Odometry::update() {
  if (encoders_ == nullptr) return;
  int32_t ticks[4];
  float wheelDeltaMm[4];
  for (uint8_t i = 0; i < 4; ++i) {
    ticks[i] = encoders_->getTicks(i);
    float ticksPerRev;
    switch (i) {
      case 0: ticksPerRev = ENCODER_TICKS_PER_OUTPUT_REV_M1; break;
      case 1: ticksPerRev = ENCODER_TICKS_PER_OUTPUT_REV_M2; break;
      case 2: ticksPerRev = ENCODER_TICKS_PER_OUTPUT_REV_M3; break;
      default: ticksPerRev = ENCODER_TICKS_PER_OUTPUT_REV_M4; break;
    }
    wheelDeltaMm[i] = ticksPerRev > 0.0f
      ? ((ticks[i] - previousTicks_[i]) * PI * WHEEL_DIAMETER_MM) / ticksPerRev
      : 0.0f;
    previousTicks_[i] = ticks[i];
  }

  const float localVx = (-wheelDeltaMm[0] + wheelDeltaMm[1] + wheelDeltaMm[2] - wheelDeltaMm[3]) / 4.0f;
  const float localVy = (wheelDeltaMm[0] + wheelDeltaMm[1] + wheelDeltaMm[2] + wheelDeltaMm[3]) / 4.0f;
  const float radius = (WHEEL_BASE_MM + TRACK_WIDTH_MM) * 0.5f;
  if (imu_ != nullptr && imu_->healthy()) {
    theta_ = imu_->getYaw() + imuHeadingOffsetDeg_;
  } else if (radius > 0.0f) {
    theta_ += ((wheelDeltaMm[0] + wheelDeltaMm[1] - wheelDeltaMm[2] - wheelDeltaMm[3]) /
               (4.0f * radius)) * 57.2957795f;
  }

  const float radians = theta_ * 0.0174532925f;
  x_ += localVx * cosf(radians) - localVy * sinf(radians);
  y_ += localVx * sinf(radians) + localVy * cosf(radians);
  pose_ = {x_, y_, theta_};
}

void Odometry::resetPose(float xMm, float yMm, float headingDeg) {
  x_ = xMm;
  y_ = yMm;
  theta_ = headingDeg;
  imuHeadingOffsetDeg_ = imu_ != nullptr && imu_->healthy()
    ? headingDeg - imu_->getYaw() : 0.0f;
  pose_ = {x_, y_, theta_};
  if (encoders_ != nullptr) {
    for (uint8_t i = 0; i < 4; ++i) previousTicks_[i] = encoders_->getTicks(i);
  }
}

float Odometry::x() const {
  return x_;
}

float Odometry::y() const {
  return y_;
}

float Odometry::theta() const {
  return theta_;
}

const robot::RobotPose& Odometry::pose() const { return pose_; }
