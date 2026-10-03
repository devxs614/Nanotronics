#include <Arduino.h>
#include "config.h"
#include "MotorController.h"
#include "EncoderManager.h"
#include "PIDController.h"
#include "MecanumDrive.h"
#include "IMUManager.h"
#include "DistanceSensors.h"
#include "ColorSensor.h"
#include "LineSensor.h"
#include "GripperController.h"
#include "DisplayManager.h"
#include "RobotStateMachine.h"
#include "MazeNavigator.h"
#include "BallHandler.h"
#include "LineAvoidanceController.h"
#include "TileNavigator.h"
#include "CheckpointManager.h"
#include "RecoveryManager.h"
#include "Diagnostics.h"
#include "VisionInterface.h"

MotorController motorController;
EncoderManager encoderManager;
PIDController wheelPid;
MecanumDrive mecanumDrive;
IMUManager imuManager;
DistanceSensors distanceSensors;
ColorSensor colorSensor;
LineSensor lineSensor;
GripperController gripperController;
DisplayManager displayManager;
RobotStateMachine robotStateMachine;
MazeNavigator mazeNavigator;
BallHandler ballHandler;
LineAvoidanceController lineAvoidanceController;
TileNavigator tileNavigator;
CheckpointManager checkpointManager;
RecoveryManager recoveryManager;
Diagnostics diagnostics;
VisionInterface visionInterface;

void setup() {
  Serial.begin(115200);
  delay(100);
  LOG_INFO("RoBorregos robot starting...");

  motorController.begin();
  encoderManager.begin();
  imuManager.begin();
  distanceSensors.begin();
  colorSensor.begin();
  lineSensor.begin();
  gripperController.begin();
  displayManager.begin();
  robotStateMachine.begin();

  displayManager.showBoot();
  LOG_INFO("Initialization complete.");
}

void loop() {
  motorController.update();
  encoderManager.update();
  imuManager.update();
  distanceSensors.update();
  colorSensor.update();
  lineSensor.update();
  gripperController.update();
  robotStateMachine.update();
  displayManager.update();

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.print(F("yaw=")); Serial.print(imuManager.getYaw());
    Serial.print(F(" rpmM1=")); Serial.print(encoderManager.getRPM(0));
    Serial.print(F(" mode=")); Serial.println(static_cast<int>(robotStateMachine.getCurrentMode()));
  }
}
