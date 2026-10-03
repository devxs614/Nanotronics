#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>
#include "config.h"

class SensorFusion {
public:
  void begin();
  void update();
  float headingDeg() const;

private:
  float headingDeg_;
};

#endif
