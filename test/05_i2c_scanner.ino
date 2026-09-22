// Prueba 5: escaner I2C
// Arduino Mega: SDA = 20, SCL = 21.

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Escaner I2C iniciado.");
}

void loop() {
  byte devices = 0;

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo en 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devices++;
    }
  }

  if (devices == 0) {
    Serial.println("No se encontraron dispositivos.");
  }

  delay(2000);
}
