#include "DisplayManager.h"

DisplayManager::DisplayManager()
  : display_(128, 64, &Wire, -1), initialized_(false), dirty_(true), lastRefreshMs_(0) {
  mode_[0] = '\0';
  sensor_[0] = '\0';
}

void DisplayManager::begin() {
  if (!ENABLE_DISPLAY) return;
  selectI2CChannel(MUX_CH_OLED);
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
  selectI2CChannel(MUX_CH_OLED);
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

void DisplayManager::drawTelemetry(Adafruit_SSD1306& target,
                                   uint16_t f, uint16_t l, uint16_t r,
                                   uint16_t red, uint16_t green, uint16_t blue,
                                   const char* yawStr, const char* colorStr,
                                   const int16_t pwm[4]) {
  // Buffers de 22 bytes: snprintf trunca a 21 caracteres (ancho de la OLED)
  char line[22];
  char dist[3][6];
  const uint16_t d[3] = {f, l, r};
  for (uint8_t i = 0; i < 3; ++i) {
    if (d[i] > 0) snprintf(dist[i], sizeof(dist[i]), "%u", d[i]);
    else strcpy(dist[i], "---");
  }

  selectI2CChannel(MUX_CH_OLED);
  target.clearDisplay();
  target.setTextSize(1);
  target.setTextColor(SSD1306_WHITE);
  target.setTextWrap(false);

  // Fila 1: "YAW:[yaw] | C:[Color]"
  snprintf(line, sizeof(line), "YAW:%s | C:%s", yawStr ? yawStr : "N/A", colorStr ? colorStr : "Desc");
  target.setCursor(0, 0);
  target.print(line);

  // Fila 2: "F:[f] L:[l] R:[r]" (mm)
  snprintf(line, sizeof(line), "F:%s L:%s R:%s", dist[0], dist[1], dist[2]);
  target.setCursor(0, 16);
  target.print(line);

  // Fila 3: "RGB:[r],[g],[b]" (crudo)
  snprintf(line, sizeof(line), "RGB:%u,%u,%u", red, green, blue);
  target.setCursor(0, 32);
  target.print(line);

  // Fila 4: "M:[m1] [m2] [m3] [m4]"
  snprintf(line, sizeof(line), "M:%d %d %d %d", pwm[0], pwm[1], pwm[2], pwm[3]);
  target.setCursor(0, 48);
  target.print(line);
  target.display();
}

void DisplayManager::renderTelemetryTo(Adafruit_SSD1306& target,
                                       uint16_t f, uint16_t l, uint16_t r,
                                       uint16_t red, uint16_t green, uint16_t blue,
                                       const char* yawStr, const char* colorStr,
                                       const int16_t pwm[4]) {
  drawTelemetry(target, f, l, r, red, green, blue, yawStr, colorStr, pwm);
}

void DisplayManager::renderTelemetry(uint16_t f, uint16_t l, uint16_t r,
                                     uint16_t red, uint16_t green, uint16_t blue,
                                     const char* yawStr, const char* colorStr,
                                     const int16_t pwm[4]) {
  if (!initialized_) return;
  drawTelemetry(display_, f, l, r, red, green, blue, yawStr, colorStr, pwm);
  lastRefreshMs_ = millis();
  dirty_ = false;
}

void DisplayManager::showBoot() {
  if (!initialized_) return;
  selectI2CChannel(MUX_CH_OLED);
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
  selectI2CChannel(MUX_CH_OLED);
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
