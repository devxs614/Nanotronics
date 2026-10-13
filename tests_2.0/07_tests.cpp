#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"
#include "ColorSensor.h"
#include "IMUManager.h"
#include "DisplayManager.h"

constexpr uint8_t I2C_MUX_ADDRESS = 0x70;
constexpr uint8_t CHANNEL_IMU   = 0;
constexpr uint8_t CHANNEL_COLOR = 1;
constexpr uint8_t CHANNEL_FRONT = 2;
constexpr uint8_t CHANNEL_RIGHT = 3;
constexpr uint8_t CHANNEL_LEFT  = 4;
constexpr uint8_t CHANNEL_OLED  = 5;

Adafruit_VL53L0X frontSensor;
Adafruit_VL53L0X rightSensor;
Adafruit_VL53L0X leftSensor;
ColorSensor color;
IMUManager imu;
DisplayManager display;

bool selectI2CChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(I2C_MUX_ADDRESS);
  Wire.write(static_cast<uint8_t>(1U << channel));
  return (Wire.endTransmission() == 0);
}

static const __FlashStringHelper* colorName(robot::ColorClass value) {
  switch (value) {
    case robot::ColorClass::COLOR_CYAN:   return F("Cian");
    case robot::ColorClass::COLOR_YELLOW: return F("Amarillo");
    case robot::ColorClass::COLOR_ORANGE: return F("Naranja");
    case robot::ColorClass::COLOR_PINK:   return F("Rosa");
    case robot::ColorClass::COLOR_RED:    return F("Rojo");
    default:                              return F("Desconocido/Verde");
  }
}

static uint16_t readRange(Adafruit_VL53L0X& sensor) {
  if (!sensor.isRangeComplete()) return 0;
  const uint16_t range = sensor.readRangeResult();
  return sensor.readRangeStatus() == 0 ? range : 0;
}

static void initializeDistance(const char* name, uint8_t channel, Adafruit_VL53L0X& sensor) {
  Serial.print(F(" -> Conectando canal "));
  Serial.print(channel);
  Serial.print(F(" (VL53L0X "));
  Serial.print(name);
  Serial.print(F(")... "));
  
  if (!selectI2CChannel(channel)) {
    Serial.println(F("[FALLO MUX]"));
    return;
  }
  
  if (sensor.begin(0x29, false)) {
    sensor.startRangeContinuous(VL53_CONTINUOUS_PERIOD_MS);
    Serial.println(F("[OK]"));
  } else {
    Serial.println(F("[SIN RESPUESTA]"));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Tiempo para estabilizar el monitor serie al reiniciar

  Serial.println(F("\n=========================================="));
  Serial.println(F("   DIAGNÓSTICO PASO A PASO I2C MUX (0x70)"));
  Serial.println(F("=========================================="));

  Wire.begin();
  Wire.setClock(100000UL);

  // 1. Verificar si el Multiplexor TCA9548A responde en 0x70
  Serial.print(F("1. Verificando MUX en 0x70... "));
  Wire.beginTransmission(I2C_MUX_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Serial.println(F("[DETECTADO]"));
  } else {
    Serial.println(F("[ERROR: MUX no responde. Revisa VCC, GND, SDA, SCL]"));
  }

  // 2. Inicializar IMU (Canal 0)
  Serial.print(F("2. Inicializando IMU (Canal 0)... "));
  if (selectI2CChannel(CHANNEL_IMU)) {
    imu.begin();
    Serial.println(F("[OK]"));
  } else {
    Serial.println(F("[ERROR MUX]"));
  }

  // 3. Inicializar Color (Canal 1)
  Serial.print(F("3. Inicializando Color (Canal 1)... "));
  if (selectI2CChannel(CHANNEL_COLOR)) {
    color.begin();
    Serial.println(F("[OK]"));
  } else {
    Serial.println(F("[ERROR MUX]"));
  }

  // 4. Inicializar Distancias (Canales 2, 3, 4)
  Serial.println(F("4. Inicializando Sensores de Distancia:"));
  initializeDistance("Frontal", CHANNEL_FRONT, frontSensor);
  initializeDistance("Derecho", CHANNEL_RIGHT, rightSensor);
  initializeDistance("Izquierdo", CHANNEL_LEFT, leftSensor);

  // 5. Inicializar OLED (Canal 5)
  Serial.print(F("5. Inicializando OLED (Canal 5)... "));
  if (selectI2CChannel(CHANNEL_OLED)) {
    display.begin();
    Serial.println(F("[OK]"));
  } else {
    Serial.println(F("[ERROR MUX]"));
  }

  Serial.println(F("\n--- INICIO DE BUCLE LOOP ---"));
}

void loop() {
  selectI2CChannel(CHANNEL_IMU);
  imu.update();
  const float yaw = imu.getYaw();

  selectI2CChannel(CHANNEL_COLOR);
  color.update();
  const robot::ColorReading& reading = color.latestReading;

  selectI2CChannel(CHANNEL_FRONT);
  const uint16_t front = readRange(frontSensor);
  selectI2CChannel(CHANNEL_RIGHT);
  const uint16_t right = readRange(rightSensor);
  selectI2CChannel(CHANNEL_LEFT);
  const uint16_t left = readRange(leftSensor);

  selectI2CChannel(CHANNEL_OLED);
  display.update();

  static uint32_t lastReportMs = 0;
  if (millis() - lastReportMs >= 200UL) {
    lastReportMs = millis();
    Serial.print(F("IMU Yaw: "));
    Serial.print(yaw, 1);
    Serial.print(F(" | Color: ["));
    Serial.print(colorName(reading.color));
    Serial.print(F("] | DistF: "));
    Serial.print(front);
    Serial.print(F(" mm | DistR: "));
    Serial.print(right);
    Serial.print(F(" mm | DistL: "));
    Serial.print(left);
    Serial.println(F(" mm"));
  }
  yield();
}