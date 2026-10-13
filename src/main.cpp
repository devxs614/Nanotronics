#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
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

Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_VL53L0X frontSensor;
Adafruit_VL53L0X rightSensor;
Adafruit_VL53L0X leftSensor;

MotorController motorController;
EncoderManager encoderManager;
MecanumDrive mecanumDrive;
IMUManager imuManager;
DistanceSensors distanceSensors;
ColorSensor color;
ColorSensor& colorSensor = color;
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

bool oledOk  = false;
bool colorOk = false;
bool fOk     = false;
bool rOk     = false;
bool lOk     = false;
bool frontOk = false;
bool rightOk = false;
bool leftOk  = false;

static const char* colorName(robot::ColorClass colorVal) {
  switch (colorVal) {
    case robot::ColorClass::COLOR_CYAN:   return "Cian";
    case robot::ColorClass::COLOR_YELLOW: return "Amarillo";
    case robot::ColorClass::COLOR_ORANGE: return "Naranja";
    case robot::ColorClass::COLOR_PINK:   return "Rosa";
    case robot::ColorClass::COLOR_RED:    return "Rojo";
    case robot::ColorClass::COLOR_NONE:   return "Ninguno";
    default:                              return "Desc";
  }
}

bool initVL53L0XWithRetry(Adafruit_VL53L0X& sensor, uint8_t channel, const char* name) {
  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    selectI2CChannel(channel);
    delay(30);
    if (sensor.begin(0x29, false)) {
      Serial.print(F("Init "));
      Serial.print(name);
      Serial.print(F(" (Canal "));
      Serial.print(channel);
      Serial.print(F("): [OK] (Intento "));
      Serial.print(attempt);
      Serial.println(F(")"));
      return true;
    }
    delay(50);
  }
  Serial.print(F("Init "));
  Serial.print(name);
  Serial.print(F(" (Canal "));
  Serial.print(channel);
  Serial.println(F("): [FALLO tras 3 intentos]"));
  return false;
}

void setup() {
  Serial.begin(115200);

#ifdef TCS_POWER
  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, HIGH);
  delay(100);
#endif

  Wire.begin();
  Wire.setClock(100000UL);

  Serial.println(F("\n=============================================="));
  Serial.println(F("   INICIALIZACIÓN TELEMETRÍA OLED + SENSORES  "));
  Serial.println(F("=============================================="));

  motorController.begin();
  encoderManager.begin();

  // Canal 5 (OLED)
  selectI2CChannel(5);
  delay(20);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.print(F("OLED SSD1306 (Canal 5): "));
  Serial.println(oledOk ? F("[OK]") : F("[FALLÓ]"));

  if (oledOk) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10);
    display.println(F("  ROBORREGOS PISTA A  "));
    display.setCursor(0, 30);
    display.println(F("Iniciando sensores..."));
    display.display();
  }

  // Canal 1 (Color)
  selectI2CChannel(1);
  delay(20);
  color.begin();
  colorOk = true;
  Serial.println(F("Color TCS34725 (Canal 1): [OK]"));

  // Canal 2 (Frontal)
  fOk = initVL53L0XWithRetry(frontSensor, 2, "Frontal");

  // Canal 3 (Derecho)
  rOk = initVL53L0XWithRetry(rightSensor, 3, "Derecho");

  // Canal 4 (Izquierdo)
  lOk = initVL53L0XWithRetry(leftSensor, 4, "Izquierdo");

  frontOk = fOk;
  rightOk = rOk;
  leftOk  = lOk;

  // Canal 0 (IMU)
  imuManager.begin();
  if (imuManager.healthy()) imuManager.calibrate();

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
  static uint32_t lastSensorReadMs = 0;
  static uint32_t lastLineUpdateMs = 0;
  static uint32_t lastOledMs = 0;
  static uint32_t lastSerialMs = 0;

  static uint16_t distF = 0;
  static uint16_t distR = 0;
  static uint16_t distL = 0;

  const uint32_t now = millis();

  // 1. Actualización rápida de Encoders y Odometría
  if (now - lastFastUpdateMs >= FAST_SENSOR_UPDATE_MS) {
    lastFastUpdateMs = now;
    encoderManager.update();
    imuManager.update();
    odometry.update();
    sensorFusion.update();
  }

  // 2. Lectura I2C separada del bucle de motores usando temporizador de 50 ms
  if (now - lastSensorReadMs >= 50UL) {
    lastSensorReadMs = now;

    // Canal 1: Color
    selectI2CChannel(1);
    delay(2);
    color.update();

    // Canal 2: Frontal
    selectI2CChannel(2);
    delay(2);
    distF = frontSensor.readRange();

    // Canal 3: Derecho
    selectI2CChannel(3);
    delay(2);
    distR = rightSensor.readRange();

    // Canal 4: Izquierdo
    selectI2CChannel(4);
    delay(2);
    distL = leftSensor.readRange();
  }

  // Variables con filtro de seguridad de rango [1..1999 mm]
  uint16_t filteredF = (distF > 0 && distF < 2000) ? distF : 0;
  uint16_t filteredL = (distL > 0 && distL < 2000) ? distL : 0;
  uint16_t filteredR = (distR > 0 && distR < 2000) ? distR : 0;

  if (now - lastLineUpdateMs >= LINE_SENSOR_UPDATE_MS) {
    lastLineUpdateMs = now;
    lineSensor.update();
    if (robotStateMachine.getCurrentMode() == robot::RobotMode::CALIBRATION) {
      lineSensor.calibrateSample();
    }
  }

  // 3. Módulos Auxiliares
  gripperController.update();
  calibrationManager.update();
  visionInterface.update();
  uint16_t markerId;
  if (visionInterface.consumeNewMarker(markerId)) displayManager.showArUco(markerId);

  // 4. Control de Motores (Actualización continua sin bloqueo)
  mecanumDrive.driveForward(100.0f);
  mecanumDrive.update();
  motorController.update();

  // 5. Refresco de Pantalla OLED (Canal 5) cada 150 ms
  if (oledOk && (now - lastOledMs >= 150UL)) {
    lastOledMs = now;

    selectI2CChannel(5);
    delay(2);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Fila 1: Encabezado
    display.setCursor(0, 0);
    display.print(F("--- TELEMETRIA ---"));

    // Fila 2: Distancias
    display.setCursor(0, 16);
    display.print(F("F:"));
    if (filteredF > 0) display.print(filteredF); else display.print(F("---"));
    display.print(F(" L:"));
    if (filteredL > 0) display.print(filteredL); else display.print(F("---"));
    display.print(F(" R:"));
    if (filteredR > 0) display.print(filteredR); else display.print(F("---"));

    // Fila 3: Componentes RGB Crudas
    display.setCursor(0, 32);
    display.print(F("RGB: "));
    display.print(color.latestReading.r);
    display.print(F(","));
    display.print(color.latestReading.g);
    display.print(F(","));
    display.print(color.latestReading.b);

    // Fila 4: Color Clasificado + Estado IMU
    display.setCursor(0, 48);
    display.print(F("Col: "));
    display.print(colorName(color.latestReading.color));
    display.print(F(" | IMU: "));
    if (imuManager.healthy()) {
      display.print(imuManager.getYaw(), 1);
    } else {
      display.print(F("N/A"));
    }

    display.display();
  }

  // 6. Reporte estructurado y limpio por Monitor Serie cada 200 ms
  if (now - lastSerialMs >= 200UL) {
    lastSerialMs = now;

    Serial.print(F("[TELEMETRÍA] Dist: F: "));
    Serial.print(filteredF);
    Serial.print(F("mm | L: "));
    Serial.print(filteredL);
    Serial.print(F("mm | R: "));
    Serial.print(filteredR);
    Serial.print(F("mm | Col: ["));
    Serial.print(colorName(color.latestReading.color));
    Serial.print(F("] (R:"));
    Serial.print(color.latestReading.r);
    Serial.print(F(" G:"));
    Serial.print(color.latestReading.g);
    Serial.print(F(" B:"));
    Serial.print(color.latestReading.b);
    Serial.print(F(") | IMU: "));
    if (imuManager.healthy()) {
      Serial.print(imuManager.getYaw(), 1);
    } else {
      Serial.print(F("N/A"));
    }
    Serial.print(F(" | M1:"));
    Serial.print(motorController.currentPWM[0]);
    Serial.print(F(" M2:"));
    Serial.print(motorController.currentPWM[1]);
    Serial.print(F(" M3:"));
    Serial.print(motorController.currentPWM[2]);
    Serial.print(F(" M4:"));
    Serial.println(motorController.currentPWM[3]);
  }
}