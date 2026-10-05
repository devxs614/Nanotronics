#include <Arduino.h>
#include "config.h"
#include "MotorController.h"
#include "MecanumDrive.h"

MotorController motors;
MecanumDrive drive;

static void runMotion(const __FlashStringHelper* name, void (*motion)(), uint16_t durationMs) {
  Serial.print(F("Kinematics: "));
  Serial.println(name);
  motion();
  const uint32_t started = millis();
  while (millis() - started < durationMs) {
    drive.update();
    motors.update();
    yield();
  }
  drive.stop();
}

static void forward() { drive.driveForward(100.0f); }
static void backward() { drive.driveBackward(100.0f); }
static void left() { drive.strafeLeft(100.0f); }
static void right() { drive.strafeRight(100.0f); }
static void clockwise() { drive.rotateCW(50.0f); }
static void counterclockwise() { drive.rotateCCW(50.0f); }

void setup() {
  Serial.begin(115200);
  motors.begin();
  drive.begin(&motors);
}

void loop() {
  runMotion(F("forward"), forward, 2000);
  runMotion(F("backward"), backward, 2000);
  runMotion(F("left"), left, 2000);
  runMotion(F("right"), right, 2000);
  runMotion(F("clockwise"), clockwise, 2000);
  runMotion(F("counterclockwise"), counterclockwise, 2000);
  drive.stop();
  motors.stopAll();
  Serial.println(F("stopped for 3 seconds"));
  const uint32_t started = millis();
  while (millis() - started < 3000UL) {
    drive.update();
    motors.update();
    yield();
  }
}
