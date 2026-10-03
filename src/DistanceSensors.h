#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"

class DistanceSensors {
public:
  void begin();
  void update();
  uint16_t readFront() const;
  uint16_t readLeft() const;
  uint16_t readRight() const;

private:
  Adafruit_VL53L0X frontSensor_;
  Adafruit_VL53L0X leftSensor_;
  Adafruit_VL53L0X rightSensor_;

  uint16_t frontMm_;
  uint16_t leftMm_;
  uint16_t rightMm_;
  bool initialized_;
};

#endif
