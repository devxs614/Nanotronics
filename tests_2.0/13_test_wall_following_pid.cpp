#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include "../src/config.h"
#include "../lib/EncoderManager/EncoderManager.h"
#include "../lib/MotorController/MotorController.h"

// Canales MUX de este test (F=2, L=3, R=4, OLED=5)
constexpr uint8_t CH_FRONT = 2;
constexpr uint8_t CH_LEFT = 3;
constexpr uint8_t CH_RIGHT = 4;
constexpr uint8_t CH_OLED = 5;

// Geometria: robot 270 x 210 mm, celda 300 x 300 mm
constexpr float CELL_MM = 280.0f;  // 28 cm reales por celda

int16_t baseSpeed = 200;
const float ERR_DEADBAND_MM = 4.0f;       // Ignorar errores laser menores a 4 mm
const uint16_t DIST_STOP_FRONT_MM = 35;   // Parada frontal al llegar a pared (3.5 cm)

// Rampa de desaceleracion y freno inverso
const float DECEL_START_MM = 230.0f;
const int16_t DECEL_END_SPEED = 80;
const int16_t BRAKE_PWM = 80;
const uint32_t BRAKE_MS = 30UL;

// Control hibrido: PD de IMU fuerte + laser suave (solo con 2 paredes)
const float IMU_KP = 5.5f;
const float IMU_KD = 1.5f;
const float LASER_WEIGHT = 0.08f;
const float CORR_LIMIT = 40.0f;
const float IMU_YAW_SIGN = 1.0f;          // Poner -1.0f si el robot se desvia mas al corregir
const float MAX_YAW_ABORT_DEG = 30.0f;    // Aborta si el rumbo se aleja demasiado

constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

const uint16_t SETPOINT_LATERAL_MM = 45;  // 4.5 cm
const uint16_t UMBRAL_PARED_MM = 150;     // Pared presente si < 150 mm

constexpr uint16_t SENSOR_MAX_MM = 2000;
constexpr uint32_t CONTROL_PERIOD_MS = 40UL;
constexpr uint32_t OLED_PERIOD_MS = 400UL;
constexpr uint32_t ADVANCE_TIMEOUT_MS = 8000UL;

Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_VL53L0X frontSensor;
Adafruit_VL53L0X leftSensor;
Adafruit_VL53L0X rightSensor;
EncoderManager encoderManager;
MotorController motorController;

bool oledOk = false;
bool frontOk = false;
bool leftOk = false;
bool rightOk = false;

// 0 = lectura invalida o fuera de rango
uint16_t distF = 0;
uint16_t distL = 0;
uint16_t distR = 0;

bool imuFound = false;
uint8_t imuAddr = MPU6050_I2C_ADDR;
float gyroBiasZ = 0.0f;
uint32_t lastImuUs = 0;
float yawActual = 0.0f;

float lastImuError = 0.0f;
float lastErr = 0.0f;
float lastCorr = 0.0f;
float advancedMm = 0.0f;

uint32_t lastSensorUpdateMs = 0;
uint32_t i2cRecoveries = 0;

void startI2C() {
  Wire.begin();
  Wire.setClock(100000UL);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);  // Cancelar transmision si I2C se congela mas de 3 ms
#endif
}

bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Reinicia el bus y comprueba que el MUX responda; reintenta hasta 3 veces
bool recoverI2C() {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    Wire.end();
    delay(2);
    startI2C();
#ifdef WIRE_HAS_TIMEOUT
    Wire.clearWireTimeoutFlag();
#endif
    if (i2cPing(0x70)) {
      i2cRecoveries++;
      return true;
    }
    delay(5);
  }
  return false;
}

// Verifica el MUX y lo recupera si no contesta
bool ensureMux() {
  if (i2cPing(0x70)) return true;
  return recoverI2C();
}

bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readGyroZRaw(int16_t& gz) {
  uint8_t b[14];
  selectI2CChannel(IMU_MUX_CHANNEL);
  Wire.beginTransmission(imuAddr);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(imuAddr, (uint8_t)14) != 14) return false;
  for (uint8_t i = 0; i < 14; i++) b[i] = Wire.read();
  gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

void initImu() {
  selectI2CChannel(IMU_MUX_CHANNEL);
  delay(5);
  const uint8_t addrs[2] = {MPU6050_I2C_ADDR, 0x69};
  for (uint8_t i = 0; i < 2 && !imuFound; i++) {
    Wire.beginTransmission(addrs[i]);
    if (Wire.endTransmission() != 0) continue;
    writeRegister(addrs[i], REG_GYRO_CONFIG, 0x18);  // +-2000 dps
    writeRegister(addrs[i], REG_PWR_MGMT_1, 0x00);   // Wake up
    delay(10);
    imuAddr = addrs[i];
    imuFound = true;
  }
  if (!imuFound) return;

  float sum = 0.0f;
  uint8_t ok = 0;
  int16_t gz;
  for (uint8_t i = 0; i < 100; i++) {
    if (readGyroZRaw(gz)) {
      sum += gz / GYRO_LSB_PER_DPS;
      ok++;
    }
    delay(5);
  }
  if (ok > 0) gyroBiasZ = sum / ok;
  lastImuUs = micros();
}

void updateYaw() {
  if (!imuFound) return;
  static uint8_t failCount = 0;
  int16_t gz;
  if (!readGyroZRaw(gz)) {
    // Reintento de conmutacion del MUX tras fallos seguidos
    if (++failCount >= 3) {
      failCount = 0;
      ensureMux();
      lastImuUs = micros();
    }
    return;
  }
  failCount = 0;
  const uint32_t nowUs = micros();
  const float dt = (nowUs - lastImuUs) / 1000000.0f;
  lastImuUs = nowUs;
  if (dt <= 0.0f || dt > 0.5f) return;
  yawActual += ((gz / GYRO_LSB_PER_DPS) - gyroBiasZ) * IMU_SCALE_FACTOR * dt;
}

bool beginSensor(Adafruit_VL53L0X& sensor, uint8_t channel) {
  selectI2CChannel(channel);
  delay(20);
  if (!sensor.begin(0x29, false)) return false;
  sensor.startRangeContinuous(33);
  return true;
}

void readSensor(Adafruit_VL53L0X& sensor, uint8_t channel, bool ok, uint16_t& dist) {
  if (!ok) {
    dist = 0;
    return;
  }
  selectI2CChannel(channel);
  if (!sensor.isRangeComplete()) return;
  const uint16_t range = sensor.readRangeResult();
  dist = (range > 0 && range < SENSOR_MAX_MM) ? range : 0;
  lastSensorUpdateMs = millis();
}

void readAllSensors() {
  readSensor(frontSensor, CH_FRONT, frontOk, distF);
  readSensor(leftSensor, CH_LEFT, leftOk, distL);
  readSensor(rightSensor, CH_RIGHT, rightOk, distR);

  // Sin lecturas nuevas por 500 ms: reintentar conmutacion del MUX y recuperar el bus
  const uint32_t now = millis();
  if ((frontOk || leftOk || rightOk) && now - lastSensorUpdateMs > 500UL) {
    lastSensorUpdateMs = now;
    if (ensureMux()) {
      selectI2CChannel(CH_FRONT);
    } else {
      Serial.println(F("[I2C] MUX no responde"));
    }
  }
}

float readAdvancedMm() {
  float sum = 0.0f;
  for (uint8_t i = 0; i < 4; i++) sum += fabs((float)encoderManager.getTicks(i));
  return (sum / 4.0f) / TICKS_PER_MM;
}

bool isFree(uint16_t dist) {
  return dist == 0 || dist > UMBRAL_PARED_MM;
}

void renderOled() {
  if (!oledOk) return;
  selectI2CChannel(CH_OLED);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(F("- PASILLO Y PAREDES -"));

  display.setCursor(0, 16);
  display.print(F("F:"));
  display.print(distF);
  display.print(F(" L:"));
  display.print(distL);
  display.print(F(" R:"));
  display.print(distR);

  display.setCursor(0, 32);
  display.print(F("F:"));
  display.print(isFree(distF) ? 'L' : 'B');
  display.print(F(" L:"));
  display.print(isFree(distL) ? 'L' : 'B');
  display.print(F(" R:"));
  display.print(isFree(distR) ? 'L' : 'B');

  display.setCursor(0, 48);
  display.print(F("AVANCE: "));
  display.print((int)advancedMm);
  display.print(F("/280mm"));

  display.display();
}

void printTelemetry() {
  Serial.print(F("[PID] F:"));
  Serial.print(distF);
  Serial.print(F(" L:"));
  Serial.print(distL);
  Serial.print(F(" R:"));
  Serial.print(distR);
  Serial.print(F(" | YAW:"));
  Serial.print(yawActual, 1);
  Serial.print(F(" ERR:"));
  Serial.print(lastErr, 1);
  Serial.print(F(" CORR:"));
  Serial.print(lastCorr, 1);
  Serial.print(F(" | AVANCE:"));
  Serial.print(advancedMm, 1);
  Serial.println(F("/280mm"));
}

void stopAllMotors() {
  motorController.stop();
  motorController.update();
}

// Desacelera en pasos cortos hasta detener los motores
void softStop(int16_t left, int16_t right) {
  for (uint8_t i = 0; i < 4; i++) {
    left = left * 2 / 3;
    right = right * 2 / 3;
    motorController.setSpeeds(left, right, left, right);
    const uint32_t t0 = millis();
    while (millis() - t0 < 25UL) motorController.update();
  }
  stopAllMotors();
}

// Freno inverso activo corto para vencer la inercia
void reverseBrake() {
  const int16_t b = -BRAKE_PWM;
  for (uint8_t i = 0; i < 4; i++) motorController.currentPWM[i] = b;
  motorController.setSpeeds(b, b, b, b);
  const uint32_t t0 = millis();
  while (millis() - t0 < BRAKE_MS) motorController.update();
  stopAllMotors();
}

void printCellState() {
  // Con motores detenidos, varias lecturas para refrescar los 3 sensores
  const uint32_t t0 = millis();
  while (millis() - t0 < 150UL) {
    readAllSensors();
    delay(5);
  }

  const bool libreF = isFree(distF);
  const bool libreL = isFree(distL);
  const bool libreR = isFree(distR);

  Serial.println(F("\n--- ESTADO DE LA CELDA ---"));
  Serial.print(F("FRENTE: ")); Serial.println(libreF ? F("LIBRE") : F("PARED"));
  Serial.print(F("IZQUIERDA: ")); Serial.println(libreL ? F("LIBRE") : F("PARED"));
  Serial.print(F("DERECHA: ")); Serial.println(libreR ? F("LIBRE") : F("PARED"));
}

void advanceOneCellPID() {
  encoderManager.resetTicks();
  lastImuError = 0.0f;
  lastErr = 0.0f;
  lastCorr = 0.0f;
  advancedMm = 0.0f;
  while (Serial.available() > 0) Serial.read();

  if (!imuFound) {
    Serial.println(F("[AVANCE] IMU no disponible, abortado"));
    return;
  }

  // Rumbo objetivo = rumbo al iniciar
  lastImuUs = micros();
  updateYaw();
  const float targetYaw = yawActual;

  Serial.println(F("\n[AVANCE] 1 casilla (280 mm) PWM 200, IMU + laser"));

  const uint32_t startMs = millis();
  uint32_t lastControlMs = 0;
  uint32_t lastLogMs = 0;
  int16_t lastLeft = baseSpeed;
  int16_t lastRight = baseSpeed;
  const __FlashStringHelper* reason = F("Odometria");
  bool activeBrake = true;
  bool zeroWarned = false;
  int16_t currentBaseSpeed = baseSpeed;

  while (true) {
    const uint32_t now = millis();

    if (Serial.available() > 0) {
      const char c = Serial.read();
      if (c == 's' || c == 'S') {
        reason = F("Usuario");
        activeBrake = false;
        break;
      }
    }
    if (now - startMs > ADVANCE_TIMEOUT_MS) {
      reason = F("TIMEOUT");
      activeBrake = false;
      break;
    }

    // La IMU se integra en cada iteracion para no perder muestras
    updateYaw();
    readAllSensors();
    advancedMm = readAdvancedMm();

    // Encoders en 0 con los motores en marcha: reintentar MUX y avisar una vez
    if (advancedMm <= 0.0f && now - startMs > 600UL) {
      if (!zeroWarned) {
        zeroWarned = true;
        Serial.println(F("[ENCODERS] 0 mm tras 600 ms, revisando bus I2C"));
        ensureMux();
      }
    } else {
      zeroWarned = false;
    }

    if (advancedMm >= CELL_MM) break;
    if (distF > 0 && distF <= DIST_STOP_FRONT_MM) {
      reason = F("Pared frontal");
      break;
    }

    const float imuError = IMU_YAW_SIGN * (yawActual - targetYaw);
    if (fabs(imuError) > MAX_YAW_ABORT_DEG) {
      reason = F("Rumbo fuera de limite");
      activeBrake = false;
      break;
    }

    // Rampa de PWM 200 a 80 en los ultimos 50 mm
    if (advancedMm >= DECEL_START_MM) {
      currentBaseSpeed = map((long)advancedMm, (long)DECEL_START_MM, (long)CELL_MM, baseSpeed, DECEL_END_SPEED);
      currentBaseSpeed = constrain(currentBaseSpeed, DECEL_END_SPEED, baseSpeed);
    } else {
      currentBaseSpeed = baseSpeed;
    }

    if (now - lastControlMs >= CONTROL_PERIOD_MS) {
      lastControlMs = now;

      const bool paredL = (distL > 0 && distL < UMBRAL_PARED_MM);
      const bool paredR = (distR > 0 && distR < UMBRAL_PARED_MM);
      float laserError = 0.0f;

      // Con un hueco lateral se ignora el laser para que no jale el chasis
      if (paredL && paredR) {
        laserError = ((float)distL - (float)distR) * LASER_WEIGHT;
      }

      const float dImuError = imuError - lastImuError;
      lastImuError = imuError;

      float correction = (imuError * IMU_KP) + (dImuError * IMU_KD) + laserError;
      correction = constrain(correction, -CORR_LIMIT, CORR_LIMIT);
      lastErr = imuError;
      lastCorr = correction;

      const int16_t left = constrain(currentBaseSpeed - (int16_t)correction, 0, 255);
      const int16_t right = constrain(currentBaseSpeed + (int16_t)correction, 0, 255);
      lastLeft = left;
      lastRight = right;
      motorController.setSpeeds(left, right, left, right);
    }

    motorController.update();

    if (now - lastLogMs >= 100UL) {
      lastLogMs = now;
      printTelemetry();
    }
    // Sin OLED durante el avance: bloquea ~100 ms el bucle y se perderian muestras de la IMU
  }

  if (activeBrake) {
    reverseBrake();
  } else {
    softStop(lastLeft, lastRight);
  }
  advancedMm = readAdvancedMm();

  Serial.print(F("[AVANCE] Finalizado por: "));
  Serial.print(reason);
  Serial.print(F(" | Distancia: "));
  Serial.print(advancedMm, 1);
  Serial.print(F(" mm | Desvio de rumbo: "));
  Serial.print(yawActual - targetYaw, 1);
  Serial.println(F(" grados"));

  printCellState();
  renderOled();
}

void printMenu() {
  Serial.println(F("\n=== TEST 13: CENTRADO PID EN PARED ==="));
  Serial.println(F("  'f' : Avanzar 1 casilla (280 mm) con PID lateral"));
  Serial.println(F("  's' : Detener motores"));
  Serial.println(F("  '?' : Menu"));
  Serial.println(F("=======================================\n"));
}

void setup() {
  Serial.begin(115200);
  startI2C();

  motorController.begin();
  encoderManager.begin();
  stopAllMotors();
  Serial.println(F("[ENCODERS] encoderManager.begin() ejecutado"));

  Serial.print(F("[MUX] 0x70: "));
  Serial.println(ensureMux() ? F("[OK]") : F("[FALLO]"));

  selectI2CChannel(CH_OLED);
  delay(20);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.print(F("[OLED] Canal 5: "));
  Serial.println(oledOk ? F("[OK]") : F("[FALLO]"));

  selectI2CChannel(IMU_MUX_CHANNEL);
  delay(5);
  const bool imuPing = i2cPing(MPU6050_I2C_ADDR);
  Serial.print(F("[IMU] Canal 0, ping 0x68: "));
  Serial.println(imuPing ? F("[OK]") : F("[FALLO]"));
  initImu();
  Serial.print(F("[IMU] Init/calibracion: "));
  Serial.println(imuFound ? F("[OK] (quieto al calibrar)") : F("[FALLO]"));

  frontOk = beginSensor(frontSensor, CH_FRONT);
  leftOk = beginSensor(leftSensor, CH_LEFT);
  rightOk = beginSensor(rightSensor, CH_RIGHT);
  Serial.print(F("[VL53L0X] F Canal 2: "));
  Serial.println(frontOk ? F("[OK]") : F("[FALLO]"));
  Serial.print(F("[VL53L0X] L Canal 3: "));
  Serial.println(leftOk ? F("[OK]") : F("[FALLO]"));
  Serial.print(F("[VL53L0X] R Canal 4: "));
  Serial.println(rightOk ? F("[OK]") : F("[FALLO]"));
  lastSensorUpdateMs = millis();

  printMenu();
  renderOled();
}

void loop() {
  static uint32_t lastOledMs = 0;
  static uint32_t lastLogMs = 0;

  readAllSensors();

  while (Serial.available() > 0) {
    const char cmd = Serial.read();
    switch (cmd) {
      case 'f': case 'F':
        advanceOneCellPID();
        break;
      case 's': case 'S':
        stopAllMotors();
        Serial.println(F("[MOTORES DETENIDOS]"));
        break;
      case '?':
        printMenu();
        break;
      default:
        break;
    }
  }

  const uint32_t now = millis();
  if (now - lastOledMs >= 250UL) {
    lastOledMs = now;
    renderOled();
  }
  if (now - lastLogMs >= 500UL) {
    lastLogMs = now;
    printTelemetry();
  }
}
