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
#include "Navigator.h"

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

bool initVL53L0XWithRetry(Adafruit_VL53L0X& sensor, uint8_t channel, const char* name) {
  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    selectI2CChannel(channel);
    delay(2);
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
    delay(5);
  }
  Serial.print(F("Init "));
  Serial.print(name);
  Serial.print(F(" (Canal "));
  Serial.print(channel);
  Serial.println(F("): [FALLO tras 3 intentos]"));
  return false;
}

bool imuFound = false;
uint8_t imuChannel = 0;
uint8_t imuAddr = 0;
uint8_t imuWhoAmI = 0;
float yawDeg = 0.0f;
float gyroBiasZ = 0.0f;
uint32_t lastImuUs = 0;

constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr float ACCEL_LSB_PER_G = 16384.0f;

bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool readImuRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t b[14];
  selectI2CChannel(MUX_CH_IMU);
  if (!readRegisters(imuAddr, REG_ACCEL_XOUT_H, b, 14)) return false;
  ax = (int16_t)((b[0] << 8) | b[1]);
  ay = (int16_t)((b[2] << 8) | b[3]);
  az = (int16_t)((b[4] << 8) | b[5]);
  gx = (int16_t)((b[8] << 8) | b[9]);
  gy = (int16_t)((b[10] << 8) | b[11]);
  gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

void scanMuxForImu() {
  const uint8_t addrs[2] = {0x68, 0x69};
  selectI2CChannel(MUX_CH_IMU);
  for (uint8_t i = 0; i < 2 && !imuFound; i++) {
    const uint8_t addr = addrs[i];
    if (!i2cPing(addr)) continue;

    uint8_t who = 0;
    readRegisters(addr, REG_WHO_AM_I, &who, 1);
    writeRegister(addr, REG_GYRO_CONFIG, 0x18);  // +-2000 dps (16.4 LSB/dps)
    writeRegister(addr, REG_PWR_MGMT_1, 0x00);

    imuFound = true;
    imuChannel = MUX_CH_IMU;
    imuAddr = addr;
    imuWhoAmI = who;
    Serial.print(F("[SCANNER OK] MPU6050 en Canal "));
    Serial.print(MUX_CH_IMU);
    Serial.print(F(", Direccion 0x"));
    Serial.print(addr, HEX);
    Serial.print(F(", WHO_AM_I = 0x"));
    Serial.println(who, HEX);
  }
  if (!imuFound) Serial.println(F("[SCANNER FALLO] MPU6050 no responde en Canal 0"));
}

// ---------------------------------------------------------------------------
// Utilidades de hardware compartidas con Navigator (yaw, odometria, OLED)
// ---------------------------------------------------------------------------
Navigator navigator;
float odometryMm = 0.0f;

float wrap180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// Integra el yaw con la IMU; devuelve el delta en grados (0 si no hubo lectura valida)
float updateYaw() {
  if (!imuFound) return 0.0f;
  int16_t ax, ay, az, gx, gy, gz;
  if (!readImuRaw(ax, ay, az, gx, gy, gz)) return 0.0f;
  const uint32_t nowUs = micros();
  const float dt = (nowUs - lastImuUs) / 1000000.0f;
  lastImuUs = nowUs;
  if (dt <= 0.0f || dt >= 0.25f) return 0.0f;
  const float delta = ((gz / GYRO_LSB_PER_DPS) - gyroBiasZ) * IMU_SCALE_FACTOR * dt;
  yawDeg = wrap180(yawDeg + delta);
  return delta;
}

float readOdometryMm() {
  float sum = 0.0f;
  for (uint8_t i = 0; i < 4; i++) sum += fabs(static_cast<float>(encoderManager.getTicks(i)));
  return (sum / 4.0f) / TICKS_PER_MM;
}

bool isWallFree(uint16_t dist) {
  return dist >= UMBRAL_PARED_MM && dist < 2000;  // dist == 0 es error de lectura = pared
}

// OLED (Canal 5): YAW|Color, distancias F/L/R, estado de paredes y odometria
void renderNavOled(uint16_t f, uint16_t l, uint16_t r, const char* colorName) {
  char line[24];
  selectI2CChannel(MUX_CH_OLED);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  snprintf(line, sizeof(line), "YAW:%d | %.10s", static_cast<int>(yawDeg), colorName);
  display.setCursor(0, 0);
  display.print(line);

  snprintf(line, sizeof(line), "F:%u L:%u R:%u", f, l, r);
  display.setCursor(0, 16);
  display.print(line);

  snprintf(line, sizeof(line), "F:%c L:%c R:%c", isWallFree(f) ? 'L' : 'B',
           isWallFree(l) ? 'L' : 'B', isWallFree(r) ? 'L' : 'B');
  display.setCursor(0, 32);
  display.print(line);

  snprintf(line, sizeof(line), "ODO: %d mm", static_cast<int>(odometryMm));
  display.setCursor(0, 48);
  display.print(line);
  display.display();
}

void setup() {
  Serial.begin(115200);

#ifdef TCS_POWER
  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, HIGH);
  delay(2);
#endif

  Wire.begin();
  Wire.setClock(100000UL);
  Wire.setWireTimeout(3000UL, true);  // Timeout de 3 ms y reinicio automático.
  Serial.println(F("\n=============================================="));
  Serial.println(F("   INICIALIZACIÓN TELEMETRÍA OLED + SENSORES  "));
  Serial.println(F("=============================================="));

  motorController.begin();
  encoderManager.begin();
  motorController.stop();

  // Canal 5 (OLED)
  selectI2CChannel(MUX_CH_OLED);
  delay(2);
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

  // Canal 1 (Color): color.begin() carga la calibracion desde la EEPROM
  selectI2CChannel(MUX_CH_COLOR);
  delay(2);
  color.begin();
  colorOk = true;
  Serial.println(F("Color TCS34725 (Canal 1): [OK]"));
  Serial.println(color.isCalibrated() ? F("Calibracion de color EEPROM: [CARGADA]")
                                      : F("Calibracion de color EEPROM: [NO ENCONTRADA]"));

  // Canal 2 (Frontal)
  Serial.print(F("Probando Canal 2 (Frontal)... "));
  fOk = initVL53L0XWithRetry(frontSensor, MUX_CH_FRONT, "Frontal");

  // Canal 3 (Izquierdo)
  Serial.print(F("Probando Canal 3 (Izquierdo)... "));
  lOk = initVL53L0XWithRetry(leftSensor, MUX_CH_LEFT, "Izquierdo");

  // Canal 4 (Derecho)
  Serial.print(F("Probando Canal 4 (Derecho)... "));
  rOk = initVL53L0XWithRetry(rightSensor, MUX_CH_RIGHT, "Derecho");
  frontOk = fOk;
  rightOk = rOk;
  leftOk  = lOk;

  // Canal 0 (IMU): direccion fija del mapa MUX; Navigator calibra el bias al iniciar.
  scanMuxForImu();
  if (imuFound) lastImuUs = micros();
  Serial.print(F("IMU MPU6050: "));
  Serial.println(imuFound ? F("[OK]") : F("[FALLÓ]"));

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
  navigator.begin();

  Serial.println(F("Robot initialized successfully"));

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
  static char yawStr[8] = "--";
  const uint32_t now = millis();

  // La navegacion inicia sola; los comandos Serial no pueden detenerla.
  while (Serial.available() > 0) {
    const char cmd = Serial.read();
    if (cmd == 'g' || cmd == 'G') {
      navigator.start();
    } else if (cmd == 's' || cmd == 'S') {
      Serial.println(F("[NAV] Comando stop ignorado; solo Rojo detiene"));
    }
  }

  // La FSM corre de inmediato desde begin(); mientras navega controla motores, sensores y OLED
  navigator.update();
  if (navigator.isRunning()) return;

  // 1. Actualización rápida de Encoders y Odometría
  if (now - lastFastUpdateMs >= FAST_SENSOR_UPDATE_MS) {
    lastFastUpdateMs = now;
    encoderManager.update();
    odometry.update();
    sensorFusion.update();
  }

  // 2. Lectura I2C separada del bucle de motores usando temporizador de 50 ms
  if (now - lastSensorReadMs >= 50UL) {
    lastSensorReadMs = now;

    // Canal 1: Color
    selectI2CChannel(MUX_CH_COLOR);
    delay(2);
    color.update();

    // Canal 2: Frontal
    selectI2CChannel(MUX_CH_FRONT);
    delay(2);
    distF = frontSensor.readRange();

    // Canal 3: Izquierdo
    selectI2CChannel(MUX_CH_LEFT);
    delay(2);
    distL = leftSensor.readRange();

    // Canal 4: Derecho
    selectI2CChannel(MUX_CH_RIGHT);
    delay(2);
    distR = rightSensor.readRange();

    // IMU por registros raw (canal detectado por el escaneo; N/A si no responde)
    if (imuFound) {
      updateYaw();
      snprintf(yawStr, sizeof(yawStr), "%d", static_cast<int>(yawDeg));
    } else {
      strcpy(yawStr, "--");
    }
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
  motorController.update();

  // 5. Refresco de Pantalla OLED (Canal 5) cada 150 ms
  if (oledOk && (now - lastOledMs >= 150UL)) {
    lastOledMs = now;

    odometryMm = readOdometryMm();
    renderNavOled(filteredF, filteredL, filteredR, color.getColorName());
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
    Serial.print(color.getColorName());
    Serial.print(F("] (R:"));
    Serial.print(color.latestReading.r);
    Serial.print(F(" G:"));
    Serial.print(color.latestReading.g);
    Serial.print(F(" B:"));
    Serial.print(color.latestReading.b);
    Serial.print(F(") | IMU: "));
    Serial.print(yawStr);
    Serial.print(F(" | E1:"));
    Serial.print(encoderManager.getTicks(0));
    Serial.print(F(" E2:"));
    Serial.print(encoderManager.getTicks(1));
    Serial.print(F(" E3:"));
    Serial.print(encoderManager.getTicks(2));
    Serial.print(F(" E4:"));
    Serial.print(encoderManager.getTicks(3));
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
