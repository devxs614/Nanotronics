#include "DisplayManager.h"

DisplayManager::DisplayManager()
  : display_(128, 64, &Wire, -1), initialized_(false), dirty_(true), lastRefreshMs_(0) {
  mode_[0] = '\0';
  sensor_[0] = '\0';
}

void DisplayManager::begin() {
  if (!ENABLE_DISPLAY) return;
  if (!display_.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    LOG_ERROR("OLED not found");
    return;
  }
  initialized_ = true;
  display_.clearDisplay();
  display_.setTextSize(1);
  display_.setTextColor(SSD1306_WHITE);
  dirty_ = true;
}

void DisplayManager::update() {
  if (!initialized_ || (!dirty_ && millis() - lastRefreshMs_ < DISPLAY_REFRESH_MS)) return;
  display_.clearDisplay();
  display_.setCursor(0, 0);
  display_.print(F("Mode: "));
  display_.println(mode_[0] ? mode_ : "READY");
  display_.print(F("Sensor: "));
  display_.println(sensor_[0] ? sensor_ : "OK");
  display_.display();
  dirty_ = false;
  lastRefreshMs_ = millis();
}

void DisplayManager::showBoot() {
  if (!initialized_) return;
  display_.clearDisplay();
  display_.setCursor(0, 0);
  display_.println(F("RoBorregos"));
  display_.println(F("CANDIDATES 2026"));
  display_.println(F("Arduino Mega"));
  display_.display();
  lastRefreshMs_ = millis();
  dirty_ = true;
}

void DisplayManager::showError(const char* text) {
  if (!initialized_) return;
  display_.clearDisplay();
  display_.setCursor(0, 0);
  display_.println(F("ERROR"));
  display_.println(text);
  display_.display();
  lastRefreshMs_ = millis();
  dirty_ = true;
}

void DisplayManager::setMode(const char* mode) {
  if (mode == nullptr || strncmp(mode_, mode, sizeof(mode_)) == 0) return;
  strncpy(mode_, mode, sizeof(mode_) - 1);
  mode_[sizeof(mode_) - 1] = '\0';
  dirty_ = true;
}

void DisplayManager::setSensor(const char* sensor) {
  if (sensor == nullptr || strncmp(sensor_, sensor, sizeof(sensor_)) == 0) return;
  strncpy(sensor_, sensor, sizeof(sensor_) - 1);
  sensor_[sizeof(sensor_) - 1] = '\0';
  dirty_ = true;
}

void DisplayManager::showColor(robot::ColorClass color) {
  setMode("COLOR");
  switch (color) {
    case robot::ColorClass::COLOR_CYAN: setSensor("CYAN"); break;
    case robot::ColorClass::COLOR_YELLOW: setSensor("YELLOW"); break;
    case robot::ColorClass::COLOR_ORANGE: setSensor("ORANGE"); break;
    case robot::ColorClass::COLOR_PINK: setSensor("PINK"); break;
    case robot::ColorClass::COLOR_RED: setSensor("RED"); break;
    default: setSensor("UNKNOWN"); break;
  }
}

void DisplayManager::showDirection(robot::ColorClass color) {
  setMode("DIRECTION");
  switch (color) {
    case robot::ColorClass::COLOR_CYAN: setSensor("RIGHT"); break;
    case robot::ColorClass::COLOR_YELLOW: setSensor("LEFT"); break;
    case robot::ColorClass::COLOR_ORANGE: setSensor("UP"); break;
    case robot::ColorClass::COLOR_PINK: setSensor("DOWN"); break;
    default: setSensor("NONE"); break;
  }
}

void DisplayManager::showArUco(uint16_t markerId) {
  setMode("ARUCO");
  char markerText[16];
  snprintf(markerText, sizeof(markerText), "ID %u", markerId);
  setSensor(markerText);
}

void DisplayManager::showStatus(const char* mode, const char* sensor) {
  setMode(mode);
  setSensor(sensor);
}
