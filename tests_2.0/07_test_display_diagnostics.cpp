#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>

#include "../src/config.h"
#include "../lib/ColorSensor/ColorSensor.h"

constexpr uint8_t MUX_ADDRESS = 0x70;
constexpr uint8_t CHANNEL_COLOR = 1;
constexpr uint8_t CHANNEL_FRONT = 2;
constexpr uint8_t CHANNEL_RIGHT = 3;
constexpr uint8_t CHANNEL_LEFT = 4;
constexpr uint8_t CHANNEL_OLED = 5;

Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_VL53L0X frontSensor;
Adafruit_VL53L0X rightSensor;
Adafruit_VL53L0X leftSensor;
ColorSensor color;

static const __FlashStringHelper* colorName(robot::ColorClass value) {
  switch (value) {
    case robot::ColorClass::COLOR_CYAN: return F("Cian");
    case robot::ColorClass::COLOR_YELLOW: return F("Amarillo");
    case robot::ColorClass::COLOR_ORANGE: return F("Naranja");
    case robot::ColorClass::COLOR_PINK: return F("Rosa");
    case robot::ColorClass::COLOR_RED: return F("Rojo");
    default: return F("Desconocido");
  }
}

static uint16_t readRange(Adafruit_VL53L0X& sensor) {
  if (!sensor.isRangeComplete()) return 0;
  const uint16_t range = sensor.readRangeResult();
  return sensor.readRangeStatus() == 0 ? range : 0;
}

static void initializeDistance(uint8_t channel, Adafruit_VL53L0X& sensor) {
  selectI2CChannel(channel);
  if (sensor.begin(0x29, false)) {
    sensor.startRangeContinuous(VL53_CONTINUOUS_PERIOD_MS);
  }
}

static void showStartup() {
  selectI2CChannel(CHANNEL_OLED);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("ROBORREGOS PISTA A"));
  display.println(F("OLED: OK (CH5)"));
  display.println(F("Color & VL53: Ready"));
  display.display();
}

static void renderTelemetry(uint16_t distF, uint16_t distL, uint16_t distR) {
  selectI2CChannel(CHANNEL_OLED);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("--- TELEMETRIA ---"));
  display.print(F("F:")); display.print(distF);
  display.print(F("mm L:")); display.print(distL);
  display.print(F(" R:")); display.print(distR);
  display.println();
  display.print(F("RGB: R:")); display.print(color.latestReading.r);
  display.print(F(" G:")); display.print(color.latestReading.g);
  display.print(F(" B:")); display.print(color.latestReading.b);
  display.println();
  display.print(F("Col: ")); display.print(colorName(color.latestReading.color));
  display.println(F(" | IMU:N/A"));
  display.display();
}

void setup() {
  Serial.begin(115200);
#ifdef TCS_POWER
  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, HIGH);
#endif
  Wire.begin();
  Wire.setClock(100000UL);

  selectI2CChannel(CHANNEL_OLED);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  showStartup();

  selectI2CChannel(CHANNEL_COLOR);
  color.begin();
  initializeDistance(CHANNEL_FRONT, frontSensor);
  initializeDistance(CHANNEL_RIGHT, rightSensor);
  initializeDistance(CHANNEL_LEFT, leftSensor);
}

void loop() {
  static uint32_t startupShownAt = millis();
  static uint32_t lastTelemetryAt = 0;

  selectI2CChannel(CHANNEL_COLOR);
  color.update();

  selectI2CChannel(CHANNEL_FRONT);
  const uint16_t distF = readRange(frontSensor);
  selectI2CChannel(CHANNEL_RIGHT);
  const uint16_t distR = readRange(rightSensor);
  selectI2CChannel(CHANNEL_LEFT);
  const uint16_t distL = readRange(leftSensor);

  if (millis() - startupShownAt >= 2000UL &&
      millis() - lastTelemetryAt >= 150UL) {
    lastTelemetryAt = millis();
    renderTelemetry(distF, distL, distR);
  }
  yield();
}
