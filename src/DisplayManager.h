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
  void setMode(const char* mode);
  void setSensor(const char* sensor);

private:
  Adafruit_SSD1306 display_;
  char mode_[16];
  char sensor_[16];
  bool initialized_;
};

#endif
