#include <Arduino.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  if (!mpu.begin(0x68)) {
    Serial.println("MPU6050 not found");
    while (1) {}
  }
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("Yaw rate:"); Serial.println(g.gyro.z * 57.2958f);
  delay(100);
}
