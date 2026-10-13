#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include "../src/config.h"
#include "../lib/ColorSensor/ColorSensor.h"

constexpr uint8_t MUX_ADDR = 0x70;
constexpr uint8_t CH_COLOR = 1;
constexpr uint8_t CH_FRONT = 2;
constexpr uint8_t CH_RIGHT = 3;
constexpr uint8_t CH_LEFT  = 4;

Adafruit_VL53L0X frontSensor;
Adafruit_VL53L0X rightSensor;
Adafruit_VL53L0X leftSensor;
ColorSensor color;

bool colorOk = false;
bool frontOk = false;
bool rightOk = false;
bool leftOk  = false;

static const __FlashStringHelper* colorName(robot::ColorClass value) {
  switch (value) {
    case robot::ColorClass::COLOR_CYAN:   return F("Cian");
    case robot::ColorClass::COLOR_YELLOW: return F("Amarillo");
    case robot::ColorClass::COLOR_ORANGE: return F("Naranja");
    case robot::ColorClass::COLOR_PINK:   return F("Rosa");
    case robot::ColorClass::COLOR_RED:    return F("Rojo");
    default:                              return F("Desconocido/Piso");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  #ifdef TCS_POWER
  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, HIGH);
  delay(100);
  #endif

  Wire.begin();
  Wire.setClock(100000UL);

  Serial.println(F("\n=============================================="));
  Serial.println(F("   DIAGNÓSTICO CANAL 4 (VL53L0X IZQUIERDO)    "));
  Serial.println(F("=============================================="));

  // 1. Color TCS34725 (Canal 1)
  selectI2CChannel(CH_COLOR);
  delay(20);
  color.begin();
  colorOk = true;

  // 2. Frontal (Canal 2)
  selectI2CChannel(CH_FRONT);
  delay(20);
  frontOk = frontSensor.begin(0x29, false);

  // 3. Derecho (Canal 3)
  selectI2CChannel(CH_RIGHT);
  delay(20);
  rightOk = rightSensor.begin(0x29, false);

  // 4. Izquierdo (Canal 4)
  selectI2CChannel(CH_LEFT);
  delay(20);
  leftOk = leftSensor.begin(0x29, false);

  // REPORTE DE ESTADO DE INICIALIZACIÓN
  Serial.print(F("Estado Canal 1 (Color):    ")); Serial.println(colorOk ? F("[OK]") : F("[FALLÓ]"));
  Serial.print(F("Estado Canal 2 (Frontal):  ")); Serial.println(frontOk ? F("[OK]") : F("[FALLÓ]"));
  Serial.print(F("Estado Canal 3 (Derecho):  ")); Serial.println(rightOk ? F("[OK]") : F("[FALLÓ]"));
  Serial.print(F("Estado Canal 4 (Izquierdo):")); Serial.println(leftOk  ? F("[OK]") : F("[FALLÓ - No responde en I2C]"));
  Serial.println(F("==============================================\n"));
}

void loop() {
  uint16_t distF = 0, distR = 0, distL = 0;
  uint8_t statusL = 0;

  // Canal 1: Color
  if (colorOk) {
    selectI2CChannel(CH_COLOR);
    delay(5);
    color.update();
  }

  // Canal 2: Frontal
  if (frontOk) {
    selectI2CChannel(CH_FRONT);
    delay(5);
    distF = frontSensor.readRange();
  }

  // Canal 3: Derecho
  if (rightOk) {
    selectI2CChannel(CH_RIGHT);
    delay(5);
    distR = rightSensor.readRange();
  }

  // Canal 4: Izquierdo
  if (leftOk) {
    selectI2CChannel(CH_LEFT);
    delay(5);
    distL = leftSensor.readRange();
    statusL = leftSensor.readRangeStatus();
  }

  static uint32_t lastReportMs = 0;
  if (millis() - lastReportMs >= 200UL) {
    lastReportMs = millis();

    Serial.print(F("Color: ["));
    Serial.print(colorName(color.latestReading.color));
    Serial.print(F("] | DistF: "));
    if (distF > 0 && distF < 2000) Serial.print(distF); else Serial.print(F("---"));
    Serial.print(F(" mm | DistR: "));
    if (distR > 0 && distR < 2000) Serial.print(distR); else Serial.print(F("---"));
    Serial.print(F(" mm | DistL: "));
    if (leftOk) {
      if (distL > 0 && distL < 2000) {
        Serial.print(distL);
      } else {
        Serial.print(F("ErrStatus:"));
        Serial.print(statusL);
      }
    } else {
      Serial.print(F("NO_INIT"));
    }
    Serial.println(F(" mm"));
  }

  yield();
}