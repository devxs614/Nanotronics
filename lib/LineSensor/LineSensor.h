#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>
#include <QTRSensors.h>
#include "config.h"

class LineSensor {
public:
  void begin();
  void update();
  bool detectWhiteLine();
  bool hasLeftLine() const;
  bool hasCenterLine() const;
  bool hasRightLine() const;
  uint16_t sensorValues[8];
  uint16_t lineCalibrationMin[8];
  uint16_t lineCalibrationMax[8];

private:
  QTRSensors qtr_;
  bool initialized_;
  robot::LineObservation lastObservation_;
};

#endif
