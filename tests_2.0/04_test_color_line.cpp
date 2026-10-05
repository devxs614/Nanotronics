#include <Arduino.h>
#include "config.h"
#include "ColorSensor.h"
#include "LineSensor.h"

ColorSensor color;
LineSensor line;
uint32_t lastReportMs = 0;

static const __FlashStringHelper* colorName(robot::ColorClass value) {
  switch (value) {
    case robot::ColorClass::COLOR_CYAN: return F("Cyan");
    case robot::ColorClass::COLOR_YELLOW: return F("Amarillo");
    case robot::ColorClass::COLOR_ORANGE: return F("Naranja");
    case robot::ColorClass::COLOR_PINK: return F("Rosa");
    case robot::ColorClass::COLOR_RED: return F("Rojo");
    default: return F("Desconocido");
  }
}

void setup() {
  Serial.begin(115200);
  color.begin();
  line.begin();
  Serial.println(F("Color TCS34725 and QTR-8A test"));
}

void loop() {
  color.update();
  line.update();
  if (millis() - lastReportMs < 100UL) {
    yield();
    return;
  }
  lastReportMs = millis();
  const robot::ColorReading& reading = color.latestReading;

  Serial.print(F("RGB normalized: "));
  Serial.print(reading.rn, 3); Serial.print(F(", "));
  Serial.print(reading.gn, 3); Serial.print(F(", "));
  Serial.print(reading.bn, 3);
  Serial.print(F(" color=")); Serial.println(colorName(reading.color));

  Serial.print(F("QTR: "));
  for (uint8_t i = 0; i < 8; ++i) {
    Serial.print(line.sensorValues[i]);
    if (i < 7) Serial.print(F(","));
  }
  Serial.println();
  if (line.detectWhiteLine()) {
    Serial.println(F("WARNING: white line detected"));
  }
}
