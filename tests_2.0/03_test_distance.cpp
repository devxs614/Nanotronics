#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"

Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft  = Adafruit_VL53L0X();

uint32_t lastReportMs = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  Wire.setClock(100000);

  // 1. MANTENER EL SENSOR DE COLOR TOTALMENTE APAGADO
  // Esto garantiza que la dirección 0x29 no esté ocupada por el TCS34725
  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, LOW);
  delay(50);

  // 2. INICIALIZAR CADA VL53L0X A TRAVES DEL MUX (Canal 2 Frontal, 3 Derecho, 4 Izquierdo)
  Serial.println(F("[Paso 1/4] Inicializando Frontal (Canal 2)..."));
  selectI2CChannel(2);
  delay(30);
  if (!loxFront.begin(0x29, false)) {
    Serial.println(F(" ERROR: Frontal no respondio"));
  } else {
    loxFront.startRangeContinuous(30);
  }

  Serial.println(F("[Paso 2/4] Inicializando Derecho (Canal 3)..."));
  selectI2CChannel(3);
  delay(30);
  if (!loxRight.begin(0x29, false)) {
    Serial.println(F(" ERROR: Derecho no respondio"));
  } else {
    loxRight.startRangeContinuous(30);
  }

  Serial.println(F("[Paso 3/4] Inicializando Izquierdo (Canal 4)..."));
  selectI2CChannel(4);
  delay(30);
  if (!loxLeft.begin(0x29, false)) {
    Serial.println(F(" ERROR: Izquierdo no respondio"));
  } else {
    loxLeft.startRangeContinuous(30);
  }
  // 4. ENCENDER EL SENSOR DE COLOR AHORA QUE 0x29 ESTÁ COMPLETAMENTE LIBRE
  Serial.println(F("[Paso 4/4] Encendiendo TCS_POWER (Sensor de color en 0x29)..."));
  digitalWrite(TCS_POWER, HIGH);
  delay(100);

  Serial.println(F("\n=== INICIALIZACIÓN SECUENCIAL FINALIZADA CON ÉXITO ==="));
}

void loop() {
  uint16_t distF = 0, distR = 0, distL = 0;

  selectI2CChannel(2);
  if (loxFront.isRangeComplete()) {
    distF = loxFront.readRangeResult();
  }
  selectI2CChannel(3);
  if (loxRight.isRangeComplete()) {
    distR = loxRight.readRangeResult();
  }
  selectI2CChannel(4);
  if (loxLeft.isRangeComplete()) {
    distL = loxLeft.readRangeResult();
  }

  const uint32_t now = millis();
  if (now - lastReportMs >= 200UL) {
    lastReportMs = now;

    Serial.print(F("Front (Ch2): "));
    if (distF > 0 && distF < 2000) { Serial.print(distF); Serial.print(F(" mm")); } else { Serial.print(F("---")); }

    Serial.print(F(" | Right (Ch3): "));
    if (distR > 0 && distR < 2000) { Serial.print(distR); Serial.print(F(" mm")); } else { Serial.print(F("---")); }

    Serial.print(F(" | Left (Ch4): "));
    if (distL > 0 && distL < 2000) { Serial.print(distL); Serial.print(F(" mm")); } else { Serial.print(F("---")); }

    Serial.println();
  }

  yield();
}