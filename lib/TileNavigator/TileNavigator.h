#ifndef TILE_NAVIGATOR_H
#define TILE_NAVIGATOR_H

#include <Arduino.h>
#include "config.h"
#include "ColorSensor.h"
#include "IMUManager.h"
#include "Odometry.h"
#include "MecanumDrive.h"
#include "DisplayManager.h"

class TileNavigator {
public:
  void begin();
  void configure(ColorSensor* color, IMUManager* imu, Odometry* odometry,
                 MecanumDrive* drive, DisplayManager* display);
  void update();
  void reset();
  bool finished() const;
  bool moving() const;

private:
  ColorSensor* color_;
  IMUManager* imu_;
  Odometry* odometry_;
  MecanumDrive* drive_;
  DisplayManager* display_;
  bool active_;
  bool finished_;
  float originYaw_;
  float targetX_;
  float targetY_;
  uint32_t lastArrivalMs_;
};

#endif
