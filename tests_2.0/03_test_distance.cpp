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

  // 2. REINICIAR Y APAGAR TODOS LOS VL53L0X VÍA XSHUT
  pinMode(DISTANCE_FRONT_XSHUT, OUTPUT);
  pinMode(DISTANCE_RIGHT_XSHUT, OUTPUT);
  pinMode(DISTANCE_LEFT_XSHUT, OUTPUT);

  digitalWrite(DISTANCE_FRONT_XSHUT, LOW);
  digitalWrite(DISTANCE_RIGHT_XSHUT, LOW);
  digitalWrite(DISTANCE_LEFT_XSHUT, LOW);
  delay(100);

  // 3. REASIGNAR CADA SENSOR UNO POR UNO (0x30, 0x31, 0x32)
  Serial.println(F("[Paso 1/4] Reasignando Frontal a 0x30..."));
  digitalWrite(DISTANCE_FRONT_XSHUT, HIGH);
  delay(50);
  if (!loxFront.begin(VL53_FRONT_ADDRESS, &Wire)) {
    Serial.println(F(" ERROR: Frontal no respondio en 0x30"));
  } else {
    loxFront.startRangeContinuous(30);
  }

  Serial.println(F("[Paso 2/4] Reasignando Derecho a 0x31..."));
  digitalWrite(DISTANCE_RIGHT_XSHUT, HIGH);
  delay(50);
  if (!loxRight.begin(VL53_RIGHT_ADDRESS, &Wire)) {
    Serial.println(F(" ERROR: Derecho no respondio en 0x31"));
  } else {
    loxRight.startRangeContinuous(30);
  }

  Serial.println(F("[Paso 3/4] Reasignando Izquierdo a 0x32..."));
  digitalWrite(DISTANCE_LEFT_XSHUT, HIGH);
  delay(50);
  if (!loxLeft.begin(VL53_LEFT_ADDRESS, &Wire)) {
    Serial.println(F(" ERROR: Izquierdo no respondio en 0x32"));
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

  if (loxFront.isRangeComplete()) {
    distF = loxFront.readRangeResult();
  }
  if (loxRight.isRangeComplete()) {
    distR = loxRight.readRangeResult();
  }
  if (loxLeft.isRangeComplete()) {
    distL = loxLeft.readRangeResult();
  }

  const uint32_t now = millis();
  if (now - lastReportMs >= 200UL) {
    lastReportMs = now;

    Serial.print(F("Front (0x30): "));
    if (distF > 0 && distF < 2000) { Serial.print(distF); Serial.print(F(" mm")); } else { Serial.print(F("---")); }

    Serial.print(F(" | Right (0x31): "));
    if (distR > 0 && distR < 2000) { Serial.print(distR); Serial.print(F(" mm")); } else { Serial.print(F("---")); }

    Serial.print(F(" | Left (0x32): "));
    if (distL > 0 && distL < 2000) { Serial.print(distL); Serial.print(F(" mm")); } else { Serial.print(F("---")); }

    Serial.println();
  }

  yield();
}