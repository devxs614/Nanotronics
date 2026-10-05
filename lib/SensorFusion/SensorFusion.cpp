#include "SensorFusion.h"

void SensorFusion::begin() {
  headingDeg_ = 0.0f;
  imu_ = nullptr;
  odometry_ = nullptr;
}

void SensorFusion::configure(IMUManager* imu, Odometry* odometry) {
  imu_ = imu;
  odometry_ = odometry;
}

void SensorFusion::update() {
  if (imu_ != nullptr && imu_->healthy()) {
    headingDeg_ = imu_->getYaw();
  } else if (odometry_ != nullptr) {
    headingDeg_ = odometry_->theta();
  }
}

float SensorFusion::headingDeg() const {
  return headingDeg_;
}
