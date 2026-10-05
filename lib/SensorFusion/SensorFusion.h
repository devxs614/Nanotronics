#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>
#include "config.h"
#include "IMUManager.h"
#include "Odometry.h"

class SensorFusion {
public:
  void begin();
  void configure(IMUManager* imu, Odometry* odometry);
  void update();
  float headingDeg() const;

private:
  IMUManager* imu_;
  Odometry* odometry_;
  float headingDeg_;
};

#endif
