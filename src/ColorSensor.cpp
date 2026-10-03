#include "ColorSensor.h"

void ColorSensor::begin() {
  // Suministrar energía al TCS34725 desde el pin D28
  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, HIGH);
  delay(100); // Pausa necesaria para estabilizar el cristal interno

  if (tcs_.begin(TCS34725_I2C_ADDRESS)) {
    initialized_ = true;
  } else {
    initialized_ = false;
  }
}

void ColorSensor::update() {
  if (!initialized_) return;
  readRaw();
  calculateNormalizedRGB();
  latestReading.color = calculateColorClassification();
}

void ColorSensor::readRaw() {
  tcs_.getRawData(&latestReading.r, &latestReading.g, &latestReading.b, &latestReading.c);
}

float ColorSensor::calculateNormalizedRGB() {
  if (latestReading.c == 0) return 0.0f;
  latestReading.rn = (float)latestReading.r / (float)latestReading.c;
  latestReading.gn = (float)latestReading.g / (float)latestReading.c;
  latestReading.bn = (float)latestReading.b / (float)latestReading.c;
  return latestReading.rn;
}

robot::ColorClass ColorSensor::calculateColorClassification() {
  // Lógica de clasificación de color por rangos normalizados
  if (latestReading.c < COLOR_CLEAR_MIN) return robot::ColorClass::COLOR_NONE;
  
  if (latestReading.rn > 0.45f && latestReading.gn < 0.35f) {
    return robot::ColorClass::COLOR_PINK;
  } else if (latestReading.rn > 0.40f && latestReading.gn > 0.38f) {
    return robot::ColorClass::COLOR_YELLOW;
  } else if (latestReading.bn > 0.40f) {
    return robot::ColorClass::COLOR_CYAN;
  }
  
  return robot::ColorClass::COLOR_UNKNOWN;
}