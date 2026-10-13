#include <Arduino.h>
#include <Wire.h>
#include "competition_config.h"
#include "MotorController.h"
#include "EncoderManager.h"
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
#include "Odometry.h"
#include "SensorFusion.h"
#include "CalibrationManager.h"

MotorController motorController;
EncoderManager encoderManager;
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
Odometry odometry;
SensorFusion sensorFusion;
CalibrationManager calibrationManager;

static const char* colorName(robot::ColorClass color) {
  switch (color) {
    case robot::ColorClass::COLOR_CYAN: return "CYAN";
    case robot::ColorClass::COLOR_YELLOW: return "YELLOW";
    case robot::ColorClass::COLOR_ORANGE: return "ORANGE";
    case robot::ColorClass::COLOR_PINK: return "PINK";
    case robot::ColorClass::COLOR_RED: return "RED";
    case robot::ColorClass::COLOR_NONE: return "NONE";
    default: return "UNKNOWN";
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000UL);
#ifdef TCS_POWER
  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, HIGH);
  delay(100);
#endif

  motorController.begin();
  encoderManager.begin();
  selectI2CChannel(MUX_CH_OLED);
  delayMicroseconds(100);
  displayManager.begin();
  displayManager.showBoot();

  imuManager.begin();
  if (imuManager.healthy()) imuManager.calibrate();
  selectI2CChannel(MUX_CH_FRONT);
  delayMicroseconds(100);
  distanceSensors.begin();
  selectI2CChannel(MUX_CH_COLOR);
  delayMicroseconds(100);
  colorSensor.begin();
  lineSensor.begin();
  gripperController.begin();

  mecanumDrive.begin(&motorController, &encoderManager);
  odometry.begin(&encoderManager, &imuManager);
  sensorFusion.begin();
  sensorFusion.configure(&imuManager, &odometry);

  checkpointManager.begin();
  checkpointManager.configure(&odometry, &imuManager);
  recoveryManager.begin();
  recoveryManager.configure(&checkpointManager, &odometry);

  ballHandler.begin();
  ballHandler.configure(&distanceSensors, &gripperController, &mecanumDrive, &odometry);
  lineAvoidanceController.begin();
  lineAvoidanceController.configure(&lineSensor, &mecanumDrive);
  tileNavigator.begin();
  tileNavigator.configure(&colorSensor, &imuManager, &odometry, &mecanumDrive, &displayManager);
  mazeNavigator.begin();
  mazeNavigator.configure(&distanceSensors, &colorSensor, &imuManager, &mecanumDrive,
                          &displayManager, &checkpointManager, &odometry);

  robotStateMachine.begin();
  robotStateMachine.configure(ACTIVE_TRACK, &mazeNavigator, &ballHandler,
                              &lineAvoidanceController, &tileNavigator, &checkpointManager,
                              &recoveryManager, &odometry, &mecanumDrive, &displayManager);

  calibrationManager.begin();
  calibrationManager.configure(&motorController, &encoderManager, &imuManager, &colorSensor,
                               &lineSensor, &gripperController);
  diagnostics.begin();
  visionInterface.begin();

  if (DEBUG_ENABLED && !COMPETITION_MODE) {
    diagnostics.printMenu();
    Serial.println(F("Robot initialized; autonomous operation does not depend on Serial."));
  }
}

void loop() {
  static uint32_t lastFastUpdateMs = 0;
  static uint32_t lastDistanceUpdateMs = 0;
  static uint32_t lastColorUpdateMs = 0;
  static uint32_t lastLineUpdateMs = 0;
  static uint32_t lastDisplayTelemetryMs = 0;
  static uint32_t lastDebugMs = 0;
  const uint32_t now = millis();

  // 1. Actualización de Sensores y Odometría
  if (now - lastFastUpdateMs >= FAST_SENSOR_UPDATE_MS) {
    lastFastUpdateMs = now;
    encoderManager.update();
    imuManager.update();
    odometry.update();
    sensorFusion.update();
  }
  if (now - lastDistanceUpdateMs >= DISTANCE_SENSOR_UPDATE_MS) {
    lastDistanceUpdateMs = now;
    distanceSensors.update();
  }
  if (now - lastColorUpdateMs >= COLOR_SENSOR_UPDATE_MS) {
    lastColorUpdateMs = now;
    selectI2CChannel(MUX_CH_COLOR);
    delayMicroseconds(100);
    colorSensor.update();
  }
  if (now - lastLineUpdateMs >= LINE_SENSOR_UPDATE_MS) {
    lastLineUpdateMs = now;
    lineSensor.update();
    if (robotStateMachine.getCurrentMode() == robot::RobotMode::CALIBRATION) {
      lineSensor.calibrateSample();
    }
  }

  // 2. Módulos Auxiliares
  gripperController.update();
  calibrationManager.update();
  visionInterface.update();
  uint16_t markerId;
  if (visionInterface.consumeNewMarker(markerId)) displayManager.showArUco(markerId);

  // 3. PRUEBA DE MOVIMIENTO DIRECTO (Fuerza avance a 100 mm/s)
  mecanumDrive.driveForward(100.0f);

  // 4. Actualización de Motores y Tracción
  mecanumDrive.update();
  motorController.update();

  if (now - lastDisplayTelemetryMs >= DISPLAY_REFRESH_MS) {
    lastDisplayTelemetryMs = now;
    selectI2CChannel(MUX_CH_OLED);
    displayManager.renderTelemetry(
        distanceSensors.readFront(), distanceSensors.readLeft(), distanceSensors.readRight(),
        colorSensor.latestReading.r, colorSensor.latestReading.g, colorSensor.latestReading.b,
        colorName(colorSensor.latestReading.color), imuManager.getYaw(), imuManager.healthy());
  }
  displayManager.update();

  if (DEBUG_ENABLED && !COMPETITION_MODE && now - lastDebugMs >= DEBUG_REFRESH_MS) {
    lastDebugMs = now;
    Serial.print(F("yaw=")); Serial.print(imuManager.getYaw());
    Serial.print(F(" rpmM1=")); Serial.print(encoderManager.getRPM(0));
    Serial.print(F(" mode=")); Serial.println(static_cast<int>(robotStateMachine.getCurrentMode()));
  }
}