#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>

// Control de pines XSHUT y alimentación
#define XSHUT_FRONT 24
#define XSHUT_RIGHT 22
#define XSHUT_LEFT  26
#define TCS_POWER   28  // Pin D28 para energizar el TCS34725

#define ADDR_FRONT 0x30
#define ADDR_RIGHT 0x31
#define ADDR_LEFT  0x32

Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft  = Adafruit_VL53L0X();
Adafruit_TCS34725 tcs     = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

bool okFront = false, okRight = false, okLeft = false, okColor = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Wire.begin();
  Wire.setClock(100000);

  // 1. APAGAR EL TCS34725 (0V en D28) Y LOS VL53L0X AL INICIO
  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, LOW); // Mantiene apagado el sensor de color

  pinMode(XSHUT_FRONT, OUTPUT); digitalWrite(XSHUT_FRONT, LOW);
  pinMode(XSHUT_RIGHT, OUTPUT); digitalWrite(XSHUT_RIGHT, LOW);
  pinMode(XSHUT_LEFT,  OUTPUT); digitalWrite(XSHUT_LEFT,  LOW);
  delay(100);

  Serial.println("\n=== INICIALIZACIÓN CON ENERGÍA DINÁMICA EN D28 ===");

  // 2. RE-DIRECCIONAR DISTANCIA (0x30, 0x31, 0x32)
  digitalWrite(XSHUT_FRONT, HIGH); delay(20);
  if (loxFront.begin(ADDR_FRONT, false, &Wire)) {
    Serial.println("[OK] Frontal -> 0x30"); okFront = true;
  } else {
    Serial.println("[FALLO] Frontal");
  }

  digitalWrite(XSHUT_RIGHT, HIGH); delay(20);
  if (loxRight.begin(ADDR_RIGHT, false, &Wire)) {
    Serial.println("[OK] Derecho -> 0x31"); okRight = true;
  } else {
    Serial.println("[FALLO] Derecho");
  }

  digitalWrite(XSHUT_LEFT, HIGH); delay(20);
  if (loxLeft.begin(ADDR_LEFT, false, &Wire)) {
    Serial.println("[OK] Izquierdo -> 0x32"); okLeft = true;
  } else {
    Serial.println("[FALLO] Izquierdo");
  }

  // 3. ENERGIZAR EL TCS34725 EN D28 Y REGISTRAR EN 0x29
  digitalWrite(TCS_POWER, HIGH); // Suministra 5V al TCS34725 desde D28
  delay(100); // Pausa para estabilizar la alimentación del sensor

  if (tcs.begin(0x29, &Wire)) {
    Serial.println("[OK] TCS34725 Color -> 0x29"); okColor = true;
  } else {
    Serial.println("[FALLO] TCS34725 Color");
  }

  Serial.println("\n--- PRUEBA EN TIEMPO REAL ---");
}

void loop() {
  VL53L0X_RangingMeasurementData_t mF, mR, mL;
  uint16_t r, g, b, c;

  Serial.print("F: ");
  if (okFront) { loxFront.rangingTest(&mF, false); Serial.print(mF.RangeMilliMeter); } else Serial.print("ERR");

  Serial.print("mm | R: ");
  if (okRight) { loxRight.rangingTest(&mR, false); Serial.print(mR.RangeMilliMeter); } else Serial.print("ERR");

  Serial.print("mm | L: ");
  if (okLeft) { loxLeft.rangingTest(&mL, false); Serial.print(mL.RangeMilliMeter); } else Serial.print("ERR");

  Serial.print("mm || Color -> ");
  if (okColor) {
    tcs.getRawData(&r, &g, &b, &c);
    Serial.print("R:"); Serial.print(r);
    Serial.print(" G:"); Serial.print(g);
    Serial.print(" B:"); Serial.print(b);
  } else {
    Serial.print("ERR");
  }

  Serial.println();
  delay(150);
}