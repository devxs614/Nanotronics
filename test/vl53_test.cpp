#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Definición de pines XSHUT (Pines del Arduino Mega)
#define XSHUT_FRONT 24
#define XSHUT_RIGHT 22
#define XSHUT_LEFT  26

// Direcciones I2C personalizadas
#define ADDR_FRONT 0x30
#define ADDR_RIGHT 0x31
#define ADDR_LEFT  0x32

Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft  = Adafruit_VL53L0X();

bool okFront = false;
bool okRight = false;
bool okLeft  = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Wire.begin();
  // Configurar velocidad de I2C estándar (100kHz)
  Wire.setClock(100000);

  Serial.println("\n=== DIAGNÓSTICO Y RESET DE SENSORES VL53L0X ===");

  // 1. FORZAR RESET DE TODOS LOS SENSORES
  pinMode(XSHUT_FRONT, OUTPUT); digitalWrite(XSHUT_FRONT, LOW);
  pinMode(XSHUT_RIGHT, OUTPUT); digitalWrite(XSHUT_RIGHT, LOW);
  pinMode(XSHUT_LEFT,  OUTPUT); digitalWrite(XSHUT_LEFT,  LOW);
  delay(100); // Dar suficiente tiempo para apagar el chip

  // 2. INICIALIZAR SENSOR FRONTAL
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(50);
  if (loxFront.begin(ADDR_FRONT)) {
    Serial.println("[OK] VL53L0X Frontal iniciado en 0x30");
    okFront = true;
  } else {
    Serial.println("[FALLO] VL53L0X Frontal no respondio");
  }

  // 3. INICIALIZAR SENSOR DERECHO
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  if (loxRight.begin(ADDR_RIGHT)) {
    Serial.println("[OK] VL53L0X Derecho iniciado en 0x31");
    okRight = true;
  } else {
    Serial.println("[FALLO] VL53L0X Derecho no respondio");
  }

  // 4. INICIALIZAR SENSOR IZQUIERDO
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  if (loxLeft.begin(ADDR_LEFT)) {
    Serial.println("[OK] VL53L0X Izquierdo iniciado en 0x32");
    okLeft = true;
  } else {
    Serial.println("[FALLO] VL53L0X Izquierdo no respondio");
  }

  Serial.println("\n--- LECTURAS EN TIEMPO REAL ---");
}

void loop() {
  VL53L0X_RangingMeasurementData_t mF, mR, mL;

  Serial.print("Frontal: ");
  if (okFront) {
    loxFront.rangingTest(&mF, false);
    if (mF.RangeStatus != 4) Serial.print(mF.RangeMilliMeter);
    else Serial.print("Fuera de rango");
  } else { Serial.print("ERR"); }

  Serial.print(" mm | Derecho: ");
  if (okRight) {
    loxRight.rangingTest(&mR, false);
    if (mR.RangeStatus != 4) Serial.print(mR.RangeMilliMeter);
    else Serial.print("Fuera de rango");
  } else { Serial.print("ERR"); }

  Serial.print(" mm | Izquierdo: ");
  if (okLeft) {
    loxLeft.rangingTest(&mL, false);
    if (mL.RangeStatus != 4) Serial.print(mL.RangeMilliMeter);
    else Serial.print("Fuera de rango");
  } else { Serial.print("ERR"); }

  Serial.println(" mm");
  delay(150);
}