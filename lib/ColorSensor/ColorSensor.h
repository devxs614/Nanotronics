#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Arduino.h>
#include <Adafruit_TCS34725.h>
#include "config.h"

class ColorSensor {
public:
  void begin();
  void update();
  void readRaw();
  void readRGB();
  float calculateNormalizedRGB();
  robot::ColorClass calculateColorClassification();

  robot::ColorReading latestReading;

private:
  Adafruit_TCS34725 tcs_;
  bool initialized_;
};

#endif
