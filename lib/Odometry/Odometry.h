#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "config.h"
#include "EncoderManager.h"
#include "IMUManager.h"

class Odometry {
public:
  void begin();
  void begin(EncoderManager* encoders, IMUManager* imu);
  void update();
  void resetPose(float xMm, float yMm, float headingDeg);
  float x() const;
  float y() const;
  float theta() const;
  const robot::RobotPose& pose() const;

private:
  EncoderManager* encoders_;
  IMUManager* imu_;
  int32_t previousTicks_[4];
  float x_;
  float y_;
  float theta_;
  float imuHeadingOffsetDeg_;
  robot::RobotPose pose_;
};

#endif
