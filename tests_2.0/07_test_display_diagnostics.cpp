#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>

#include "../src/config.h"
#include "../lib/ColorSensor/ColorSensor.h"

constexpr uint8_t MUX_ADDR = 0x70;
constexpr uint8_t CH_IMU   = 0;
constexpr uint8_t CH_COLOR = 1;
constexpr uint8_t CH_FRONT = 2;
constexpr uint8_t CH_RIGHT = 3;
constexpr uint8_t CH_LEFT  = 4;
constexpr uint8_t CH_OLED  = 5;

Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_VL53L0X frontSensor;
Adafruit_VL53L0X rightSensor;
Adafruit_VL53L0X leftSensor;
ColorSensor color;

bool oledOk  = false;
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
    default:                              return F("Desc");
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
  Serial.println(F("   INICIALIZACIÓN TELEMETRÍA OLED + SENSORES  "));
  Serial.println(F("=============================================="));

  // 1. OLED (Canal 5)
  selectI2CChannel(CH_OLED);
  delay(20);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.print(F("OLED SSD1306 (Canal 5): "));
  Serial.println(oledOk ? F("[OK]") : F("[FALLÓ]"));

  if (oledOk) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10);
    display.println(F("  ROBORREGOS PISTA A  "));
    display.setCursor(0, 30);
    display.println(F("Iniciando sensores..."));
    display.display();
  }

  // 2. Color (Canal 1)
  selectI2CChannel(CH_COLOR);
  delay(20);
  color.begin();
  colorOk = true;
  Serial.println(F("Color TCS34725 (Canal 1): [OK]"));

  // 3. VL53L0X Frontal (Canal 2)
  selectI2CChannel(CH_FRONT);
  delay(20);
  frontOk = frontSensor.begin(0x29, false);
  Serial.print(F("VL53L0X Frontal (Canal 2): "));
  Serial.println(frontOk ? F("[OK]") : F("[FALLÓ]"));

  // 4. VL53L0X Derecho (Canal 3)
  selectI2CChannel(CH_RIGHT);
  delay(20);
  rightOk = rightSensor.begin(0x29, false);
  Serial.print(F("VL53L0X Derecho (Canal 3): "));
  Serial.println(rightOk ? F("[OK]") : F("[FALLÓ]"));

  // 5. VL53L0X Izquierdo (Canal 4)
  selectI2CChannel(CH_LEFT);
  delay(20);
  leftOk = leftSensor.begin(0x29, false);
  Serial.print(F("VL53L0X Izquierdo (Canal 4): "));
  Serial.println(leftOk ? F("[OK]") : F("[FALLÓ]"));

  Serial.println(F("==============================================\n"));
  delay(1000);
}

void loop() {
  uint16_t distF = 0, distR = 0, distL = 0;

  // Lectura Color (Canal 1)
  if (colorOk) {
    selectI2CChannel(CH_COLOR);
    delay(5);
    color.update();
  }

  // Lectura Frontal (Canal 2)
  if (frontOk) {
    selectI2CChannel(CH_FRONT);
    delay(5);
    distF = frontSensor.readRange();
  }

  // Lectura Derecho (Canal 3)
  if (rightOk) {
    selectI2CChannel(CH_RIGHT);
    delay(5);
    distR = rightSensor.readRange();
  }

  // Lectura Izquierdo (Canal 4)
  if (leftOk) {
    selectI2CChannel(CH_LEFT);
    delay(5);
    distL = leftSensor.readRange();
  }

  // Refrescar Pantalla OLED (Canal 5) cada 150 ms
  static uint32_t lastOledMs = 0;
  if (oledOk && (millis() - lastOledMs >= 150UL)) {
    lastOledMs = millis();

    selectI2CChannel(CH_OLED);
    delay(2);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Fila 1: Encabezado
    display.setCursor(0, 0);
    display.print(F("--- TELEMETRIA ---"));

    // Fila 2: Distancias
    display.setCursor(0, 16);
    display.print(F("F:"));
    if (distF > 0 && distF < 2000) display.print(distF); else display.print(F("---"));
    display.print(F(" L:"));
    if (distL > 0 && distL < 2000) display.print(distL); else display.print(F("---"));
    display.print(F(" R:"));
    if (distR > 0 && distR < 2000) display.print(distR); else display.print(F("---"));

    // Fila 3: Componentes RGB Crudas
    display.setCursor(0, 32);
    display.print(F("RGB: "));
    display.print(color.latestReading.r);
    display.print(F(","));
    display.print(color.latestReading.g);
    display.print(F(","));
    display.print(color.latestReading.b);

    // Fila 4: Color Clasificado + Estado IMU
    display.setCursor(0, 48);
    display.print(F("Col: "));
    display.print(colorName(color.latestReading.color));
    display.print(F(" | IMU: N/A"));

    display.display();
  }

  // Reporte en Monitor Serie
  static uint32_t lastSerialMs = 0;
  if (millis() - lastSerialMs >= 200UL) {
    lastSerialMs = millis();

    Serial.print(F("DistF: ")); Serial.print(distF);
    Serial.print(F(" mm | DistR: ")); Serial.print(distR);
    Serial.print(F(" mm | DistL: ")); Serial.print(distL);
    Serial.print(F(" mm | RGB: ("));
    Serial.print(color.latestReading.r); Serial.print(F(","));
    Serial.print(color.latestReading.g); Serial.print(F(","));
    Serial.print(color.latestReading.b); Serial.println(F(")"));
  }

  yield();
}