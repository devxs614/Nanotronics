#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "../src/config.h"
#include "../lib/EncoderManager/EncoderManager.h"
#include "../lib/MotorController/MotorController.h"

constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr int16_t CAL_SPIN_PWM = 42;

constexpr int16_t TURN_PWM = 50;
constexpr int16_t SLOW_PWM = 30;
constexpr int16_t BRAKE_PWM = 45;
constexpr float SLOW_ZONE_DEG = 15.0f;
constexpr uint32_t BRAKE_MS = 35UL;
constexpr uint32_t TURN_TIMEOUT_MS = 6000UL;

Adafruit_SSD1306 display(128, 64, &Wire, -1);
EncoderManager encoderManager;
MotorController motorController;

bool oledOk = false;
bool imuFound = false;
uint8_t imuAddr = 0;
float gyroBiasZ = 0.0f;
uint32_t lastImuUs = 0;

float yawDeg = 0.0f;
float targetDeg = 0.0f;
bool turning = false;

bool oledDirty = true;

float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

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
  selectI2CChannel(IMU_MUX_CHANNEL);
  if (!readRegisters(imuAddr, REG_ACCEL_XOUT_H, b, 14)) return false;
  ax = (int16_t)((b[0] << 8) | b[1]);
  ay = (int16_t)((b[2] << 8) | b[3]);
  az = (int16_t)((b[4] << 8) | b[5]);
  gx = (int16_t)((b[8] << 8) | b[9]);
  gy = (int16_t)((b[10] << 8) | b[11]);
  gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

void findAndWakeImu() {
  selectI2CChannel(IMU_MUX_CHANNEL);
  delay(5);
  const uint8_t addrs[2] = {MPU6050_I2C_ADDR, 0x69};
  for (uint8_t i = 0; i < 2 && !imuFound; i++) {
    if (!i2cPing(addrs[i])) continue;
    writeRegister(addrs[i], REG_GYRO_CONFIG, 0x18);  // GYRO_CONFIG = +-2000 dps
    writeRegister(addrs[i], REG_PWR_MGMT_1, 0x00);   // Wake up
    delay(10);
    imuAddr = addrs[i];
    imuFound = true;
  }
  Serial.print(F("[IMU] Canal 0: "));
  if (imuFound) {
    Serial.print(F("OK, direccion 0x"));
    Serial.println(imuAddr, HEX);
  } else {
    Serial.println(F("NO responde"));
  }
}

void calibrateGyroBias() {
  float sum = 0.0f;
  uint8_t ok = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (uint8_t i = 0; i < 100; i++) {
    if (readImuRaw(ax, ay, az, gx, gy, gz)) {
      sum += gz / GYRO_LSB_PER_DPS;
      ok++;
    }
    delay(5);
  }
  if (ok > 0) gyroBiasZ = sum / ok;
  Serial.print(F("[IMU] Bias gyro Z (dps): "));
  Serial.println(gyroBiasZ, 3);
}

void updateYaw() {
  if (!imuFound) return;
  int16_t ax, ay, az, gx, gy, gz;
  if (!readImuRaw(ax, ay, az, gx, gy, gz)) return;
  const uint32_t nowUs = micros();
  const float dt = (nowUs - lastImuUs) / 1000000.0f;
  lastImuUs = nowUs;
  if (dt <= 0.0f || dt > 0.5f) return;
  yawDeg += (gz / GYRO_LSB_PER_DPS - gyroBiasZ) * IMU_SCALE_FACTOR * dt;
}

void renderOled() {
  if (!oledOk) return;
  selectI2CChannel(MUX_CH_OLED);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(F("-- TEST GIROS IMU --"));

  display.setCursor(0, 16);
  display.print(F("YAW ACTUAL: "));
  if (imuFound) display.print(wrap180(yawDeg), 1); else display.print(F("--"));

  display.setCursor(0, 32);
  display.print(F("OBJETIVO: "));
  display.print(wrap180(targetDeg), 1);

  display.setCursor(0, 48);
  display.print(F("ESTADO: "));
  display.print(turning ? F("Girando") : F("Detenido"));

  display.display();
}

void renderOledTurnProgress(float turnedAngle, float targetDegrees) {
  static uint32_t lastMs = 0;
  const uint32_t now = millis();
  if (now - lastMs < 250UL) return;
  lastMs = now;

  Serial.print(F("[GIRO] Acumulado: "));
  Serial.print(turnedAngle, 1);
  Serial.print(F(" / "));
  Serial.println(targetDegrees, 1);

  if (!oledOk) return;
  selectI2CChannel(MUX_CH_OLED);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("-- TEST GIROS IMU --"));
  display.setCursor(0, 16);
  display.print(F("YAW ACTUAL: "));
  display.print(wrap180(yawDeg), 1);
  display.setCursor(0, 32);
  display.print(F("OBJETIVO: "));
  display.print(targetDegrees, 1);
  display.setCursor(0, 48);
  display.print(F("ESTADO: Girando"));
  display.display();
}

void setAllMotors(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
  motorController.setSpeeds(m1, m2, m3, m4);
  motorController.update();
}

void stopAllMotors() {
  motorController.stop();
}

// Giro izquierda: M1/M3 atras, M2/M4 adelante. Derecha: el inverso.
void setTurnPwm(bool turnLeft, int16_t pwm) {
  const int16_t p = turnLeft ? pwm : -pwm;
  setAllMotors(-p, p, -p, p);
}

void executeImuTurn(bool turnLeft, float targetDegrees = 90.0f) {
  if (!imuFound) {
    Serial.println(F("[GIRO] IMU no disponible"));
    return;
  }

  // Canal del MUX de la IMU una sola vez; dentro del bucle no se toca la OLED
  selectI2CChannel(IMU_MUX_CHANNEL);
  delay(2);

  float turnedAngle = 0.0f;
  bool slow = false;
  turning = true;
  targetDeg = targetDegrees;

  Serial.print(F("[GIRO] "));
  Serial.print(turnLeft ? F("IZQUIERDA") : F("DERECHA"));
  Serial.print(F(" "));
  Serial.print(targetDegrees, 1);
  Serial.println(F(" grados"));

  const uint32_t startMs = millis();
  uint32_t lastTurnUs = micros();
  setTurnPwm(turnLeft, TURN_PWM);

  while (turnedAngle < targetDegrees) {
    if (millis() - startMs > TURN_TIMEOUT_MS) {
      Serial.println(F("[GIRO] TIMEOUT"));
      break;
    }
    if (Serial.available() > 0 && (Serial.peek() == 's' || Serial.peek() == 'S')) {
      Serial.read();
      Serial.println(F("[GIRO] Abortado"));
      break;
    }

    uint8_t b[14];
    Wire.beginTransmission(imuAddr);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(imuAddr, (uint8_t)14) == 14) {
      for (uint8_t i = 0; i < 14; i++) b[i] = Wire.read();
      const int16_t gz = (int16_t)((b[12] << 8) | b[13]);

      const uint32_t nowUs = micros();
      const float dt = (nowUs - lastTurnUs) / 1000000.0f;
      lastTurnUs = nowUs;

      const float gzd = ((gz / GYRO_LSB_PER_DPS) - gyroBiasZ) * IMU_SCALE_FACTOR;
      yawDeg += gzd * dt;
      turnedAngle += fabs(gzd * dt);
    }

    if (!slow && turnedAngle >= (targetDegrees - SLOW_ZONE_DEG)) {
      slow = true;
      setTurnPwm(turnLeft, SLOW_PWM);
    }
    motorController.update();
    yield();
  }

  // Freno activo inverso corto para contrarrestar la inercia
  setTurnPwm(!turnLeft, BRAKE_PWM);
  for (uint8_t i = 0; i < 4; i++) motorController.currentPWM[i] = motorController.targetPWM[i];
  motorController.update();
  delay(BRAKE_MS);
  stopAllMotors();

  turning = false;
  lastImuUs = micros();

  Serial.print(F("Giro completado | Ángulo acumulado: "));
  Serial.print(turnedAngle, 2);
  Serial.print(F("° | Error: "));
  Serial.print(turnedAngle - targetDegrees, 2);
  Serial.println(F("°"));

  renderOled();
}

// Integra el giro hasta que llegue cualquier tecla por Serial; motorsOn gira el chasis con motores
float measureSpinUntilKey(bool motorsOn) {
  float turnedAngle = 0.0f;

  selectI2CChannel(IMU_MUX_CHANNEL);
  delay(2);
  while (Serial.available() > 0) Serial.read();

  if (motorsOn) setAllMotors(CAL_SPIN_PWM, -CAL_SPIN_PWM, CAL_SPIN_PWM, -CAL_SPIN_PWM);
  uint32_t lastTurnUs = micros();

  while (Serial.available() == 0) {
    uint8_t b[14];
    Wire.beginTransmission(imuAddr);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(imuAddr, (uint8_t)14) == 14) {
      for (uint8_t i = 0; i < 14; i++) b[i] = Wire.read();
      const int16_t gz = (int16_t)((b[12] << 8) | b[13]);

      const uint32_t nowUs = micros();
      const float dt = (nowUs - lastTurnUs) / 1000000.0f;
      lastTurnUs = nowUs;

      const float gzd = ((gz / GYRO_LSB_PER_DPS) - gyroBiasZ) * IMU_SCALE_FACTOR;
      turnedAngle += fabs(gzd * dt);
    }
    if (motorsOn) motorController.update();
    yield();
  }

  if (motorsOn) stopAllMotors();
  while (Serial.available() > 0) Serial.read();
  lastImuUs = micros();
  return turnedAngle;
}

void printCalibrationResult(float turnedAngle) {
  Serial.println(F("\n--- RESULTADO DE LA PRUEBA DE GIRO ---"));
  Serial.print(F("Grados medidos por la IMU: "));
  Serial.print(turnedAngle, 2);
  Serial.println(F("° (esperados: 360.00°)"));

  const float rawCalculated = turnedAngle / IMU_SCALE_FACTOR;
  const float suggestedFactor = (rawCalculated > 0.1f) ? (360.0f / rawCalculated) : 1.0f;

  Serial.print(F(">>> FACTOR DE ESCALA EXACTO SUGERIDO (IMU_SCALE_FACTOR): "));
  Serial.println(suggestedFactor, 4);
  Serial.println(F("---------------------------------------------------\n"));
}

void runMotorSpinCalibration() {
  if (!imuFound) {
    Serial.println(F("[GIRO] IMU no disponible"));
    return;
  }

  Serial.println(F("\n--- INICIANDO GIRO CON MOTORES (PWM 42) ---"));
  Serial.println(F("El robot esta girando sobre su eje."));
  Serial.println(F(">>> PRESIONA CUALQUIER TECLA EN SERIAL CUANDO COMPLETE 1 VUELTA (360°) <<<"));

  turning = true;
  const float turnedAngle = measureSpinUntilKey(true);
  turning = false;

  printCalibrationResult(turnedAngle);
  renderOled();
}

void runManualSpinCalibration() {
  if (!imuFound) {
    Serial.println(F("[CAL] IMU no disponible"));
    return;
  }

  Serial.println(F("\n--- CALIBRACION MANUAL (SIN MOTORES) ---"));
  Serial.println(F("Gira el robot a mano sobre su eje."));
  Serial.println(F(">>> PRESIONA CUALQUIER TECLA CUANDO COMPLETES 1 VUELTA (360°) <<<"));

  const float turnedAngle = measureSpinUntilKey(false);

  printCalibrationResult(turnedAngle);
  renderOled();
}

void printMenu() {
  Serial.println(F("\n=== TEST 12: GIROS IMU +-90 ==="));
  Serial.println(F("  'g' : Giro con motores, tecla al completar 360 (calibra factor)"));
  Serial.println(F("  'c' : Calibracion manual girando a mano 360"));
  Serial.println(F("  'l' : Girar 90 grados a la IZQUIERDA (antihorario)"));
  Serial.println(F("  'r' : Girar 90 grados a la DERECHA (horario)"));
  Serial.println(F("  'z' : Yaw relativo a 0.0"));
  Serial.println(F("  's' : Detener motores"));
  Serial.println(F("================================\n"));
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'g': case 'G':
      runMotorSpinCalibration();
      break;
    case 'c': case 'C':
      runManualSpinCalibration();
      break;
    case 'l': case 'L':
      executeImuTurn(true, 90.0f);
      break;
    case 'r': case 'R':
      executeImuTurn(false, 90.0f);
      break;
    case 'z': case 'Z':
      yawDeg = 0.0f;
      targetDeg = 0.0f;
      Serial.println(F("[YAW] Reiniciado a 0.0"));
      oledDirty = true;
      break;
    case 's': case 'S':
      stopAllMotors();
      turning = false;
      Serial.println(F("[MOTORES DETENIDOS]"));
      oledDirty = true;
      break;
    case '?':
      printMenu();
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000UL);

  motorController.begin();
  encoderManager.begin();
  stopAllMotors();

  selectI2CChannel(MUX_CH_OLED);
  delay(20);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.println(oledOk ? F("[OLED] OK en Canal 5") : F("[OLED] FALLO en Canal 5"));

  findAndWakeImu();
  if (imuFound) {
    Serial.println(F("[IMU] Calibrando bias, mantener quieto..."));
    calibrateGyroBias();
    lastImuUs = micros();
  }

  printMenu();
  renderOled();
}

void loop() {
  static uint32_t lastOledMs = 0;
  static uint32_t lastLogMs = 0;

  updateYaw();

  while (Serial.available() > 0) handleCommand(Serial.read());

  motorController.update();

  const uint32_t now = millis();

  if (oledDirty || now - lastOledMs >= 250UL) {
    lastOledMs = now;
    oledDirty = false;
    renderOled();
  }

  if (now - lastLogMs >= 200UL) {
    lastLogMs = now;
    Serial.print(F("[YAW] "));
    Serial.print(wrap180(yawDeg), 1);
    Serial.print(F(" | Obj: "));
    Serial.print(wrap180(targetDeg), 1);
    Serial.print(F(" | "));
    Serial.println(turning ? F("Girando") : F("Detenido"));
  }
}
