#include "DisplayManager.h"

DisplayManager::DisplayManager()
  : display_(128, 64, &Wire, -1), initialized_(false) {
  mode_[0] = '\0';
  sensor_[0] = '\0';
}

void DisplayManager::begin() {
  if (!display_.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    LOG_ERROR("OLED not found");
    return;
  }
  initialized_ = true;
  display_.clearDisplay();
  display_.setTextSize(1);
  display_.setTextColor(SSD1306_WHITE);
}

void DisplayManager::update() {
  if (!initialized_) return;
  display_.clearDisplay();
  display_.setCursor(0, 0);
  display_.print(F("Mode: "));
  display_.println(mode_[0] ? mode_ : "READY");
  display_.print(F("Sensor: "));
  display_.println(sensor_[0] ? sensor_ : "OK");
  display_.display();
}

void DisplayManager::showBoot() {
  if (!initialized_) return;
  display_.clearDisplay();
  display_.setCursor(0, 0);
  display_.println(F("RoBorregos"));
  display_.println(F("CANDIDATES 2026"));
  display_.println(F("Arduino Mega"));
  display_.display();
}

void DisplayManager::showError(const char* text) {
  if (!initialized_) return;
  display_.clearDisplay();
  display_.setCursor(0, 0);
  display_.println(F("ERROR"));
  display_.println(text);
  display_.display();
}

void DisplayManager::setMode(const char* mode) {
  strncpy(mode_, mode, sizeof(mode_) - 1);
  mode_[sizeof(mode_) - 1] = '\0';
}

void DisplayManager::setSensor(const char* sensor) {
  strncpy(sensor_, sensor, sizeof(sensor_) - 1);
  sensor_[sizeof(sensor_) - 1] = '\0';
}
