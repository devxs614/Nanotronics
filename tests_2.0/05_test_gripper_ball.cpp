#include <Arduino.h>
#include "config.h"
#include "DistanceSensors.h"
#include "GripperController.h"
#include "MotorController.h"
#include "MecanumDrive.h"

DistanceSensors distances;
GripperController gripper;
MotorController motors;
MecanumDrive drive;

static void captureBall() {
  gripper.open();
  Serial.println(F("Approaching ball"));
  while (true) {
    distances.update();
    const uint16_t front = distances.readFront();
    Serial.print(F("distance=")); Serial.println(front);
    if (distances.frontHealthy() && front <= BALL_CAPTURE_DISTANCE_MM) {
      drive.stop();
      gripper.close();
      Serial.println(F("Ball captured"));
      return;
    }
    drive.driveForward(BALL_SEARCH_SPEED_MM_S);
    drive.update();
    gripper.update();
    motors.update();
    yield();
  }
}

void setup() {
  Serial.begin(115200);
  motors.begin();
  drive.begin(&motors);
  distances.begin();
  gripper.begin();
  Serial.println(F("Commands: o=open c=close h=hold b=capture"));
}

void loop() {
  distances.update();
  gripper.update();
  drive.update();
  motors.update();
  if (Serial.available()) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'o') gripper.open();
    else if (command == 'c') gripper.close();
    else if (command == 'h') gripper.hold();
    else if (command == 'b') captureBall();
  }
  yield();
}
