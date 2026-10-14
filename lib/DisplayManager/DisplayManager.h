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
  // Dashboard de 4 filas, max 21 chars por fila (yawStr = "N/A" si la IMU no responde)
  void renderTelemetry(uint16_t f, uint16_t l, uint16_t r,
                       uint16_t red, uint16_t green, uint16_t blue,
                       const char* yawStr, const char* colorStr, const int16_t pwm[4]);
  static void renderTelemetryTo(Adafruit_SSD1306& target,
                                uint16_t f, uint16_t l, uint16_t r,
                                uint16_t red, uint16_t green, uint16_t blue,
                                const char* yawStr, const char* colorStr, const int16_t pwm[4]);
  void showBoot();
  void showError(const char* text);
  void showColor(robot::ColorClass color);
  void showDirection(robot::ColorClass color);
  void showArUco(uint16_t markerId);
  void setMode(const char* mode);
  void setSensor(const char* sensor);
  void showStatus(const char* mode, const char* sensor);

private:
  static void drawTelemetry(Adafruit_SSD1306& target,
                            uint16_t f, uint16_t l, uint16_t r,
                            uint16_t red, uint16_t green, uint16_t blue,
                            const char* yawStr, const char* colorStr, const int16_t pwm[4]);
  Adafruit_SSD1306 display_;
  char mode_[16];
  char sensor_[16];
  bool initialized_;
  bool dirty_;
  uint32_t lastRefreshMs_;
};

#endif
