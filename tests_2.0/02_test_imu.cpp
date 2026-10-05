#include <Arduino.h>
#include "config.h"
#include "MotorController.h"
#include "MecanumDrive.h"
#include "IMUManager.h"
#include "PIDController.h"

MotorController motors;
MecanumDrive drive;
IMUManager imu;
PIDController headingPid;
float initialHeading = 0.0f;
uint32_t lastReportMs = 0;

void setup() {
  Serial.begin(115200);
  motors.begin();
  drive.begin(&motors);
  imu.begin();
  initialHeading = imu.getYaw();
  headingPid.setTunings(KP_HEADING, KI_HEADING, KD_HEADING);
  headingPid.setOutputLimits(-80.0f, 80.0f);
  headingPid.setSetpoint(initialHeading);
  headingPid.enable(true);
  Serial.println(F("IMU test: MPU6050 at 0x68/0x69"));
}

void loop() {
  imu.update();
  drive.update();
  motors.update();
  const float yaw = imu.getYaw();
  const float correction = imu.healthy() ? headingPid.update(yaw) : 0.0f;
  drive.drive(100.0f, 0.0f, correction);

  if (millis() - lastReportMs >= 100UL) {
    lastReportMs = millis();
    Serial.print(F("yaw=")); Serial.print(yaw, 2);
    Serial.print(F(" pitch=")); Serial.print(imu.getPitch(), 2);
    Serial.print(F(" roll=")); Serial.print(imu.getRoll(), 2);
    Serial.print(F(" correction=")); Serial.println(correction, 2);
  }
  yield();
}
