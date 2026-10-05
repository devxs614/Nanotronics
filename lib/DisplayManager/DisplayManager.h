#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"

class DisplayManager {
public:
  DisplayManager();
  void begin();
  void update();
  void showBoot();
  void showError(const char* text);
  void showColor(robot::ColorClass color);
  void showDirection(robot::ColorClass color);
  void showArUco(uint16_t markerId);
  void setMode(const char* mode);
  void setSensor(const char* sensor);
  void showStatus(const char* mode, const char* sensor);

private:
  Adafruit_SSD1306 display_;
  char mode_[16];
  char sensor_[16];
  bool initialized_;
  bool dirty_;
  uint32_t lastRefreshMs_;
};

#endif
