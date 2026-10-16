#include <Arduino.h>
#include <Wire.h>
#include "../src/config.h"
#include "../lib/EncoderManager/EncoderManager.h"
#include "../lib/MotorController/MotorController.h"

// Test aislado de giros: no conmuta el MUX durante el giro, salvo Canal 0 IMU.
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr int16_t TURN_PWM = 40;
constexpr int16_t SLOW_PWM = 22;
constexpr int16_t BRAKE_PWM = 45;
constexpr float SLOW_ZONE_DEG = 15.0f;
constexpr uint32_t TURN_TIMEOUT_MS = 6000UL;

MotorController motorController;
EncoderManager encoderManager;
bool imuFound = false;
uint8_t imuAddr = MPU6050_I2C_ADDR;
float gyroBiasZ = 0.0f;
float imuScaleFactor = IMU_SCALE_FACTOR;
uint16_t brakeMs = 35;
float yawDeg = 0.0f;

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readGyroZ(int16_t& gz) {
  uint8_t data[14];
  Wire.beginTransmission(imuAddr);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(imuAddr, static_cast<uint8_t>(14)) != 14) return false;
  for (uint8_t i = 0; i < 14; ++i) data[i] = Wire.read();
  gz = static_cast<int16_t>((data[12] << 8) | data[13]);
  return true;
}

bool initImu() {
  selectI2CChannel(IMU_MUX_CHANNEL);
  const uint8_t addresses[] = {MPU6050_I2C_ADDR, 0x69};
  for (uint8_t address : addresses) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() != 0) continue;
    if (!writeRegister(address, REG_GYRO_CONFIG, 0x18) ||
        !writeRegister(address, REG_PWR_MGMT_1, 0x00)) continue;
    imuAddr = address;
    imuFound = true;
    return true;
  }
  return false;
}

void calibrateBias() {
  float sum = 0.0f;
  uint8_t valid = 0;
  int16_t gz = 0;
  for (uint8_t i = 0; i < 100; ++i) {
    if (readGyroZ(gz)) { sum += static_cast<float>(gz) / GYRO_LSB_PER_DPS; ++valid; }
    delay(2);
  }
  gyroBiasZ = valid ? sum / valid : 0.0f;
}

void setTurnPwm(bool left, int16_t pwm) {
  const int16_t direction = left ? pwm : -pwm;
  motorController.setSpeeds(-direction, direction, -direction, direction);
  motorController.update();
}

void stopWithBrake(bool turnLeft) {
  setTurnPwm(!turnLeft, BRAKE_PWM);
  delay(brakeMs);
  motorController.stop();
  motorController.update();
  delay(100);
}

void executeImuTurn(bool turnLeft, float targetDegrees) {
  if (!imuFound) { Serial.println(F("[GIRO] IMU no disponible")); return; }
  selectI2CChannel(IMU_MUX_CHANNEL);  // Canal 0 fijo durante todo el giro.
  delay(2);
  float turnedAngle = 0.0f;
  bool slow = false;
  uint32_t lastUs = micros();
  const uint32_t startedMs = millis();
  setTurnPwm(turnLeft, TURN_PWM);

  while (turnedAngle < targetDegrees && millis() - startedMs < TURN_TIMEOUT_MS) {
    int16_t gz = 0;
    if (readGyroZ(gz)) {
      const uint32_t nowUs = micros();
      const float dt = (nowUs - lastUs) * 0.000001f;
      lastUs = nowUs;
      if (dt > 0.0f && dt < 0.25f) {
        const float delta = ((static_cast<float>(gz) / GYRO_LSB_PER_DPS) - gyroBiasZ) * imuScaleFactor * dt;
        yawDeg += delta;
        turnedAngle += fabs(delta);
      }
    }
    if (!slow && turnedAngle >= targetDegrees - SLOW_ZONE_DEG) {
      slow = true;
      setTurnPwm(turnLeft, SLOW_PWM);
    }
    motorController.update();
    yield();
  }

  stopWithBrake(turnLeft);
  const float signedTarget = turnLeft ? -targetDegrees : targetDegrees;
  const float signedMeasured = turnLeft ? -turnedAngle : turnedAngle;
  Serial.println(F("--- RESULTADO GIRO IMU ---"));
  Serial.print(F("Angulo objetivo: ")); Serial.print(signedTarget, 2); Serial.println(F(" grados"));
  Serial.print(F("Angulo IMU: ")); Serial.print(signedMeasured, 2); Serial.println(F(" grados"));
  Serial.print(F("Error angular: ")); Serial.print(signedMeasured - signedTarget, 2); Serial.println(F(" grados"));
  Serial.print(F("Tiempo: ")); Serial.print(millis() - startedMs); Serial.println(F(" ms"));
}

void run360Calibration() {
  if (!imuFound) { Serial.println(F("[CAL] IMU no disponible")); return; }
  selectI2CChannel(IMU_MUX_CHANNEL);
  while (Serial.available()) Serial.read();
  Serial.println(F("[CAL] Giro 360 activo; presiona cualquier tecla al completar una vuelta."));
  setTurnPwm(false, TURN_PWM);
  float rawDegrees = 0.0f;
  uint32_t lastUs = micros();
  while (!Serial.available()) {
    int16_t gz = 0;
    if (readGyroZ(gz)) {
      const uint32_t nowUs = micros();
      const float dt = (nowUs - lastUs) * 0.000001f;
      lastUs = nowUs;
      if (dt > 0.0f && dt < 0.25f) rawDegrees += fabs((static_cast<float>(gz) / GYRO_LSB_PER_DPS - gyroBiasZ) * dt);
    }
    motorController.update();
    yield();
  }
  while (Serial.available()) Serial.read();
  stopWithBrake(false);
  const float suggested = rawDegrees > 0.1f ? 360.0f / rawDegrees : 0.0f;
  Serial.print(F("[CAL] Raw degrees: ")); Serial.println(rawDegrees, 3);
  Serial.print(F("[CAL] IMU_SCALE_FACTOR sugerido (360/rawDegrees): ")); Serial.println(suggested, 4);
}

void printMenu() {
  Serial.println(F("\n=== TEST 16: CALIBRACION GIRO IMU ==="));
  Serial.println(F("c: 360 hasta tecla | l: -90 izquierda | r: +90 derecha | u: 180"));
  Serial.println(F("+/-: escala +/-0.1 | b/B: freno +/ - 1 ms (20..60) | ?: menu"));
  Serial.print(F("Escala=")); Serial.print(imuScaleFactor, 4);
  Serial.print(F(" BrakeMs=")); Serial.println(brakeMs);
}

void handleCommand(char command) {
  switch (command) {
    case 'c': case 'C': run360Calibration(); return;
    case 'l': case 'L': executeImuTurn(true, 90.0f); return;
    case 'r': case 'R': executeImuTurn(false, 90.0f); return;
    case 'u': case 'U': executeImuTurn(true, 180.0f); return;
    case '+': imuScaleFactor += 0.1f; break;
    case '-': imuScaleFactor = max(0.1f, imuScaleFactor - 0.1f); break;
    case 'b': if (brakeMs < 60) ++brakeMs; break;
    case 'B': if (brakeMs > 20) --brakeMs; break;
    case '?': printMenu(); return;
    default: return;
  }
  Serial.print(F("[AJUSTE] Escala=")); Serial.print(imuScaleFactor, 3);
  Serial.print(F(" BrakeMs=")); Serial.println(brakeMs);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000UL);
  Wire.setWireTimeout(3000UL, true);
  motorController.begin(); encoderManager.begin(); motorController.stop();
  if (initImu()) calibrateBias();
  Serial.println(imuFound ? F("[IMU] OK") : F("[IMU] FALLO"));
  printMenu();
}

void loop() {
  if (Serial.available()) handleCommand(static_cast<char>(Serial.read()));
  delay(5);
}
