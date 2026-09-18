#include <Arduino.h>
#include <Servo.h>

Servo gripper;

void setup() {
  gripper.attach(5);
  Serial.begin(115200);
}

void loop() {
  gripper.write(20);
  delay(500);
  gripper.write(120);
  delay(500);
}
