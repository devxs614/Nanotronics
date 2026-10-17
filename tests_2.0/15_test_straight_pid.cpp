#include <Arduino.h>
#include <Wire.h>
#include "../src/config.h"
#include "../lib/EncoderManager/EncoderManager.h"
#include "../lib/MotorController/MotorController.h"

// Test aislado: avance recto de una celda con PD sobre yaw del MPU6050.
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr float CELL_DISTANCE_MM = 280.0f;
constexpr int16_t BASE_PWM = 100;
constexpr int16_t BRAKE_PWM = 60;
constexpr uint32_t BRAKE_MS = 35UL;
constexpr uint32_t ADVANCE_TIMEOUT_MS = 8000UL;

MotorController motorController;
EncoderManager encoderManager;

bool imuFound = false;
uint8_t imuAddr = MPU6050_I2C_ADDR;
float gyroBiasZ = 0.0f;
float yawDeg = 0.0f;
uint32_t lastYawUs = 0;
float kp = 4.0f;
float kd = 0.0f;

float wrap180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readGyroZ(int16_t& gz) {
  uint8_t sample[14];
  selectI2CChannel(IMU_MUX_CHANNEL);
  Wire.beginTransmission(imuAddr);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(imuAddr, static_cast<uint8_t>(14)) != 14) return false;
  for (uint8_t i = 0; i < 14; ++i) sample[i] = Wire.read();
  gz = static_cast<int16_t>((sample[12] << 8) | sample[13]);
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
    break;
  }
  return imuFound;
}

void calibrateGyroBias() {
  float total = 0.0f;
  uint8_t valid = 0;
  int16_t gz = 0;
  for (uint8_t i = 0; i < 100; ++i) {
    if (readGyroZ(gz)) {
      total += static_cast<float>(gz) / GYRO_LSB_PER_DPS;
      ++valid;
    }
    delay(2);
  }
  gyroBiasZ = valid ? total / valid : 0.0f;
  lastYawUs = micros();
}

void updateYaw() {
  int16_t gz = 0;
  if (!imuFound || !readGyroZ(gz)) return;
  const uint32_t nowUs = micros();
  const float dt = (nowUs - lastYawUs) * 0.000001f;
  lastYawUs = nowUs;
  if (dt <= 0.0f || dt > 0.25f) return;
  const float dps = (static_cast<float>(gz) / GYRO_LSB_PER_DPS - gyroBiasZ) * IMU_SCALE_FACTOR;
  yawDeg = wrap180(yawDeg + dps * dt);
}

float readOdometryMm() {
  float ticks = 0.0f;
  for (uint8_t motor = 0; motor < 4; ++motor) ticks += fabs(static_cast<float>(encoderManager.getTicks(motor)));
  return (ticks / 4.0f) / TICKS_PER_MM;
}

void setForward(int16_t leftPwm, int16_t rightPwm) {
  motorController.setSpeeds(leftPwm, rightPwm, leftPwm, rightPwm);
  motorController.update();
}

void stopWithBrake() {
  motorController.setSpeeds(-BRAKE_PWM, -BRAKE_PWM, -BRAKE_PWM, -BRAKE_PWM);
  motorController.update();
  delay(BRAKE_MS);
  motorController.stop();
  motorController.update();
}

void runStraightPid() {
  if (!imuFound) {
    Serial.println(F("[PID] IMU no disponible"));
    return;
  }

  motorController.stop();
  delay(100);
  updateYaw();
  const float targetYaw = yawDeg;
  float lastYawError = 0.0f;
  uint32_t lastControlUs = micros();
  encoderManager.resetTicks();

  Serial.println(F("[PID] Avance 280 mm iniciado"));
  const uint32_t startedMs = millis();
  uint32_t lastReportMs = 0;
  float distanceMm = 0.0f;
  while (distanceMm < CELL_DISTANCE_MM && millis() - startedMs < ADVANCE_TIMEOUT_MS) {
    updateYaw();
    const uint32_t nowUs = micros();
    const float dt = (nowUs - lastControlUs) * 0.000001f;
    lastControlUs = nowUs;
    const float yawError = wrap180(yawDeg - targetYaw);
    const float derivative = dt > 0.0005f ? (yawError - lastYawError) / dt : 0.0f;
    float correction = kp * yawError + kd * derivative;
    correction = constrain(correction, -60.0f, 60.0f);
    // El signo se invierte respecto de la primera prueba: yaw positivo debe
    // sumar al lado izquierdo y restar al derecho para cerrar el lazo.
    const int16_t leftPwm = constrain(BASE_PWM + static_cast<int16_t>(correction), 40, 160);
    const int16_t rightPwm = constrain(BASE_PWM - static_cast<int16_t>(correction), 40, 160);
    setForward(leftPwm, rightPwm);
    lastYawError = yawError;
    distanceMm = readOdometryMm();

    if (millis() - lastReportMs >= 100UL) {
      lastReportMs = millis();
      Serial.print(F("[PID] ODO:")); Serial.print(distanceMm, 1);
      Serial.print(F(" Yaw:")); Serial.print(yawDeg, 2);
      Serial.print(F(" Err:")); Serial.print(yawError, 2);
      Serial.print(F(" L/R:")); Serial.print(leftPwm); Serial.print('/'); Serial.println(rightPwm);
    }
    yield();
  }

  stopWithBrake();
  updateYaw();
  const float finalError = wrap180(yawDeg - targetYaw);
  Serial.println(F("--- RESULTADO AVANCE PID ---"));
  Serial.print(F("Odometria alcanzada: ")); Serial.print(readOdometryMm(), 1); Serial.println(F(" mm"));
  Serial.print(F("Yaw objetivo: ")); Serial.print(targetYaw, 2); Serial.println(F(" grados"));
  Serial.print(F("Yaw final: ")); Serial.print(yawDeg, 2); Serial.println(F(" grados"));
  Serial.print(F("Error angular final: ")); Serial.print(finalError, 2); Serial.println(F(" grados"));
}

void printMenu() {
  Serial.println(F("\n=== TEST 15: AVANCE RECTO PID ==="));
  Serial.println(F("f: avanzar 280 mm | p/P: Kp +/ - 0.5 | d/D: Kd +/ - 0.1"));
  Serial.println(F("z: yaw=0 | ?: menu"));
  Serial.print(F("Kp=")); Serial.print(kp, 2); Serial.print(F(" Kd=")); Serial.println(kd, 2);
}

void handleCommand(char command) {
  switch (command) {
    case 'f': case 'F': runStraightPid(); break;
    case 'p': kp += 0.5f; break;
    case 'P': kp = max(0.0f, kp - 0.5f); break;
    case 'd': kd += 0.1f; break;
    case 'D': kd = max(0.0f, kd - 0.1f); break;
    case 'z': case 'Z': yawDeg = 0.0f; Serial.println(F("[PID] Yaw reiniciado")); return;
    case '?': printMenu(); return;
    default: return;
  }
  Serial.print(F("[PID] Kp=")); Serial.print(kp, 2); Serial.print(F(" Kd=")); Serial.println(kd, 2);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000UL);
  Wire.setWireTimeout(3000UL, true);
  motorController.begin();
  encoderManager.begin();
  motorController.stop();
  if (initImu()) calibrateGyroBias();
  Serial.println(imuFound ? F("[IMU] OK") : F("[IMU] FALLO"));
  printMenu();
}

void loop() {
  updateYaw();
  if (Serial.available()) handleCommand(static_cast<char>(Serial.read()));
  delay(5);
}
