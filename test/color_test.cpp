#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define XSHUT_FRONT 24
#define XSHUT_RIGHT 22
#define XSHUT_LEFT  26

Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft  = Adafruit_VL53L0X();

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial) { ; }

  // Re-direccionar VL53L0X antes de escanear
  pinMode(XSHUT_FRONT, OUTPUT); digitalWrite(XSHUT_FRONT, LOW);
  pinMode(XSHUT_RIGHT, OUTPUT); digitalWrite(XSHUT_RIGHT, LOW);
  pinMode(XSHUT_LEFT,  OUTPUT); digitalWrite(XSHUT_LEFT,  LOW);
  delay(10);

  digitalWrite(XSHUT_FRONT, HIGH); delay(10); loxFront.begin(0x30);
  digitalWrite(XSHUT_RIGHT, HIGH); delay(10); loxRight.begin(0x31);
  digitalWrite(XSHUT_LEFT,  HIGH); delay(10); loxLeft.begin(0x32);

  Serial.println("\n=== ESCÁNER I2C CON DIRECCIONES RE-ASIGNADAS ===");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Escaneando bus I2C...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo en 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);

      if (address == 0x29) Serial.print(" -> Color TCS34725");
      else if (address == 0x30) Serial.print(" -> VL53L0X Frontal");
      else if (address == 0x31) Serial.print(" -> VL53L0X Derecho");
      else if (address == 0x32) Serial.print(" -> VL53L0X Izquierdo");
      else if (address == 0x3C) Serial.print(" -> Pantalla OLED");
      else if (address == 0x68) Serial.print(" -> IMU MPU6050");
      Serial.println();

      nDevices++;
    }
  }
  Serial.print("Dispositivos detectados: "); Serial.println(nDevices);
  Serial.println();
  delay(3000);
}