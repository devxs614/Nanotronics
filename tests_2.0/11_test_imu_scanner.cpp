#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "../src/config.h"

constexpr uint8_t MUX_ADDR = 0x70;
constexpr uint8_t OLED_CHANNEL = 5;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

constexpr float ACCEL_LSB_PER_G = 16384.0f;  // +-2 g
constexpr float GYRO_LSB_PER_DPS = 131.0f;   // +-250 dps
constexpr float COMPLEMENTARY_ALPHA = 0.98f;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

bool oledOk = false;
bool imuFound = false;
uint8_t imuChannel = 0;
uint8_t imuAddr = 0;
uint8_t imuWhoAmI = 0;

float yawDeg = 0.0f;
float pitchDeg = 0.0f;
float rollDeg = 0.0f;
float gyroBiasZ = 0.0f;
uint32_t lastImuUs = 0;

void selectMuxChannel(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delay(2);
}

bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
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

bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

void printHex(uint8_t value) {
  Serial.print(F("0x"));
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

void printHexDisplay(uint8_t value) {
  display.print(F("0x"));
  if (value < 0x10) display.print('0');
  display.print(value, HEX);
}

bool readImuRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t b[14];
  selectMuxChannel(imuChannel);
  if (!readRegisters(imuAddr, REG_ACCEL_XOUT_H, b, 14)) return false;
  ax = (int16_t)((b[0] << 8) | b[1]);
  ay = (int16_t)((b[2] << 8) | b[3]);
  az = (int16_t)((b[4] << 8) | b[5]);
  gx = (int16_t)((b[8] << 8) | b[9]);
  gy = (int16_t)((b[10] << 8) | b[11]);
  gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

void calibrateGyroBias() {
  const uint8_t samples = 100;
  float sum = 0.0f;
  uint8_t ok = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (uint8_t i = 0; i < samples; i++) {
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

void scanMuxForImu() {
  Serial.println(F("\n=== ESCANEO MUX PARA MPU6050 (0x68 / 0x69) ==="));
  const uint8_t addrs[2] = {0x68, 0x69};

  for (uint8_t ch = 0; ch < 8; ch++) {
    selectMuxChannel(ch);
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t addr = addrs[i];
      if (!i2cPing(addr)) continue;

      uint8_t who = 0;
      bool whoOk = readRegisters(addr, REG_WHO_AM_I, &who, 1);
      bool wakeOk = writeRegister(addr, REG_PWR_MGMT_1, 0x00);

      Serial.print(F("[SCANNER OK] MPU6050 encontrado en Canal "));
      Serial.print(ch);
      Serial.print(F(", Direccion "));
      printHex(addr);
      Serial.print(F(", WHO_AM_I = "));
      if (whoOk) printHex(who); else Serial.print(F("ERR"));
      Serial.print(F(" (esperado 0x68)"));
      Serial.println(wakeOk ? F(" | Sleep OFF") : F(" | ERROR despertando"));

      if (!imuFound) {
        imuFound = true;
        imuChannel = ch;
        imuAddr = addr;
        imuWhoAmI = who;
      }
    }
  }

  if (!imuFound) {
    Serial.println(F("[SCANNER FALLO] MPU6050 no responde en ningun canal (0-7)."));
  }
}

void renderOled() {
  if (!oledOk) return;
  selectMuxChannel(OLED_CHANNEL);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(F("-- TEST MPU6050 --"));

  display.setCursor(0, 16);
  if (imuFound) {
    display.print(F("CH:"));
    display.print(imuChannel);
    display.print(F(" | ADDR:"));
    printHexDisplay(imuAddr);
  } else {
    display.print(F("CH:-- | ADDR:--"));
  }

  display.setCursor(0, 32);
  display.print(F("WHO_AM_I: "));
  if (imuFound) printHexDisplay(imuWhoAmI); else display.print(F("--"));

  display.setCursor(0, 48);
  display.print(F("YAW: "));
  if (imuFound) display.print(yawDeg, 1); else display.print(F("--"));

  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin();
  Wire.setClock(100000);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);
#endif

  selectMuxChannel(OLED_CHANNEL);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.println(oledOk ? F("[OLED] OK en Canal 5") : F("[OLED] FALLO en Canal 5"));

  selectMuxChannel(0);
  Serial.println(F("\n--- BARRIDO I2C COMPLETO EN CANAL 0 ---"));
  for (uint8_t a = 1; a < 127; a++) {
    if (a == MUX_ADDR) continue;
    if (i2cPing(a)) {
      Serial.print(F("Dispositivo detectado en Canal 0 -> Dirección: 0x"));
      if (a < 16) Serial.print("0");
      Serial.println(a, HEX);
    }
  }
  Serial.println(F("----------------------------------------\n"));

  scanMuxForImu();

  if (imuFound) {
    Serial.println(F("[IMU] Calibrando bias del giroscopio, mantener quieto..."));
    calibrateGyroBias();
    lastImuUs = micros();
  }

  renderOled();
}

void loop() {
  static uint32_t lastMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastMs < 100UL) return;
  lastMs = nowMs;

  if (!imuFound) {
    Serial.println(F("[IMU] No detectada. Reinicia tras revisar cableado/canal."));
    renderOled();
    return;
  }

  int16_t ax, ay, az, gx, gy, gz;
  if (!readImuRaw(ax, ay, az, gx, gy, gz)) {
    Serial.println(F("[IMU] Error de lectura I2C"));
    renderOled();
    return;
  }

  uint32_t nowUs = micros();
  float dt = (nowUs - lastImuUs) / 1000000.0f;
  lastImuUs = nowUs;

  float axg = ax / ACCEL_LSB_PER_G;
  float ayg = ay / ACCEL_LSB_PER_G;
  float azg = az / ACCEL_LSB_PER_G;
  float gxd = gx / GYRO_LSB_PER_DPS;
  float gyd = gy / GYRO_LSB_PER_DPS;
  float gzd = gz / GYRO_LSB_PER_DPS - gyroBiasZ;

  float accRoll = atan2(ayg, azg) * RAD_TO_DEG;
  float accPitch = atan2(-axg, sqrt(ayg * ayg + azg * azg)) * RAD_TO_DEG;

  rollDeg = COMPLEMENTARY_ALPHA * (rollDeg + gxd * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accRoll;
  pitchDeg = COMPLEMENTARY_ALPHA * (pitchDeg + gyd * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accPitch;

  yawDeg += gzd * dt;
  if (yawDeg > 180.0f) yawDeg -= 360.0f;
  else if (yawDeg < -180.0f) yawDeg += 360.0f;

  Serial.print(F("[IMU] CH:"));
  Serial.print(imuChannel);
  Serial.print(F(" ADDR:"));
  printHex(imuAddr);
  Serial.print(F(" WHO:"));
  printHex(imuWhoAmI);
  Serial.print(F(" | Yaw:"));
  Serial.print(yawDeg, 1);
  Serial.print(F(" Pitch:"));
  Serial.print(pitchDeg, 1);
  Serial.print(F(" Roll:"));
  Serial.println(rollDeg, 1);

  renderOled();
}
