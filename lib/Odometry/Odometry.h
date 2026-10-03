#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "config.h"

class Odometry {
public:
  void begin();
  void update();
  float x() const;
  float y() const;
  float theta() const;

private:
  float x_;
  float y_;
  float theta_;
};

#endif
