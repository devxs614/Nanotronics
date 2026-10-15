#include "Navigator.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"
#include "MotorController.h"
#include "EncoderManager.h"
#include "ColorSensor.h"

// Hardware y utilidades compartidas definidas en main.cpp
extern Adafruit_VL53L0X frontSensor;
extern Adafruit_VL53L0X rightSensor;
extern Adafruit_VL53L0X leftSensor;
extern MotorController motorController;
extern EncoderManager encoderManager;
extern ColorSensor color;
extern bool oledOk;
extern bool fOk;
extern bool rOk;
extern bool lOk;
extern bool imuFound;
extern float yawDeg;
extern float gyroBiasZ;
extern uint32_t lastImuUs;
extern float odometryMm;
extern Adafruit_SSD1306 display;
extern Navigator navigator;

float wrap180(float angle);
float updateYaw();
bool readImuRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz);
float readOdometryMm();
bool isWallFree(uint16_t dist);

namespace {

constexpr int16_t NAV_BASE_PWM = 200;
constexpr float NAV_DECEL_START_MM = CELL_MM - 50.0f;
constexpr int16_t NAV_DECEL_END_PWM = 80;
constexpr int16_t NAV_BRAKE_PWM = 80;
constexpr uint32_t NAV_BRAKE_MS = 30UL;
constexpr uint16_t NAV_STOP_FRONT_MM = 35;
constexpr float NAV_YAW_LOCK_KP = 3.0f;
constexpr float NAV_CORR_LIMIT = 40.0f;
constexpr float NAV_MAX_YAW_ABORT_DEG = 30.0f;
constexpr uint32_t NAV_TIMEOUT_MS = 8000UL;

// Parametros validados en el test de giros IMU.
constexpr int16_t TURN_PWM = 50;
constexpr int16_t SLOW_PWM = 30;
constexpr int16_t BRAKE_PWM = 45;
constexpr float SLOW_ZONE_DEG = 15.0f;
constexpr uint32_t BRAKE_MS = 35UL;
constexpr uint32_t TURN_TIMEOUT_MS = 6000UL;
constexpr uint8_t I2C_MUX_ADDRESS = 0x70;
constexpr uint8_t TOF_I2C_ADDRESS = 0x29;
constexpr uint16_t TOF_INVALID_MM = 8000;

struct CellStatus {
  bool freeFront;
  bool freeLeft;
  bool freeRight;
};

uint16_t scanDistF = 0;
uint16_t scanDistL = 0;
uint16_t scanDistR = 0;
uint8_t simultaneousTofZeroCycles = 0;

void resetMuxChannel(uint8_t channel) {
  selectI2CChannel(channel);
}

bool reinitializeTof(Adafruit_VL53L0X& sensor, uint8_t channel) {
  selectI2CChannel(channel);
  if (!sensor.begin(TOF_I2C_ADDRESS, false, &Wire)) return false;
  delayMicroseconds(250);
  return true;
}

// Cada llamada toma una conversion fisica individual. Nunca devuelve un dato
// anterior: una muestra ausente, invalida, fuera de rango o con timeout es 0.
uint16_t readDistanceSensor(Adafruit_VL53L0X& sensor, uint8_t channel, bool ok) {
  if (!ok) return 0;

  const auto takeFreshMeasurement = [&]() -> uint16_t {
    selectI2CChannel(channel);
    const uint16_t range = sensor.readRange();
    const uint8_t status = sensor.readRangeStatus();
    return (range > 0 && range < TOF_INVALID_MM && status == 0) ? range : 0;
  };

  uint16_t range = takeFreshMeasurement();
  if (range != 0) return range;

  // Un 0, timeout o distancia invalida reinicia solo el VL53L0X de este canal
  // y exige una segunda conversion fisica antes de declarar pared.
  Serial.println(F("[TOF] Reinicializando canal"));
  if (!reinitializeTof(sensor, channel)) return 0;
  range = takeFreshMeasurement();
  if (range == 0) Serial.println(F("[TOF] Fallo -> B"));
  return range;
}

void recoverI2cBus() {
  Serial.println(F("[I2C] Recuperacion global"));
  Wire.end();
  delay(5);
  Wire.begin();
  Wire.setClock(100000UL);
  Wire.setWireTimeout(3000UL, true);

  // Reinicia explicitamente el estado del TCA9548A antes de los nuevos barridos.
  Wire.beginTransmission(I2C_MUX_ADDRESS);
  Wire.write(static_cast<uint8_t>(0x00));
  Wire.endTransmission();
  delayMicroseconds(100);

  // Wire.end() tambien desconecta logicamente a la OLED. Restablece el
  // controlador y fuerza el mismo render de cuatro filas usado por la FSM.
  selectI2CChannel(MUX_CH_OLED);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOk) {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    navigator.renderOled();
  }
}

void recoverTofBusIfNeeded(uint16_t& distF, uint16_t& distL, uint16_t& distR) {
  if (distF != 0 || distL != 0 || distR != 0) {
    simultaneousTofZeroCycles = 0;
    return;
  }

  if (++simultaneousTofZeroCycles < 2) return;
  simultaneousTofZeroCycles = 0;
  recoverI2cBus();

  // Tras restaurar Wire y el MUX, toma tres mediciones nuevas, no almacenadas.
  distF = readDistanceSensor(frontSensor, MUX_CH_FRONT, fOk);
  distL = readDistanceSensor(leftSensor, MUX_CH_LEFT, lOk);
  distR = readDistanceSensor(rightSensor, MUX_CH_RIGHT, rOk);
}

// Lee F (canal 2), L (canal 3) y R (canal 4). Libre solo si 150 <= dist < 2000;
// dist == 0 es un error de lectura y cuenta como PARED.
CellStatus scanCellWalls() {
  scanDistF = readDistanceSensor(frontSensor, MUX_CH_FRONT, fOk);
  scanDistL = readDistanceSensor(leftSensor, MUX_CH_LEFT, lOk);
  scanDistR = readDistanceSensor(rightSensor, MUX_CH_RIGHT, rOk);
  recoverTofBusIfNeeded(scanDistF, scanDistL, scanDistR);

  CellStatus status;
  status.freeFront = (scanDistF >= UMBRAL_PARED_MM && scanDistF < 2000);
  status.freeLeft = (scanDistL >= UMBRAL_PARED_MM && scanDistL < 2000);
  status.freeRight = (scanDistR >= UMBRAL_PARED_MM && scanDistR < 2000);
  return status;
}

void printCellStatus(const CellStatus& status) {
  Serial.println(F("\n--- ESTADO DE LA CELDA ---"));
  Serial.print(F("FRENTE: ")); Serial.println(status.freeFront ? F("LIBRE") : F("PARED"));
  Serial.print(F("IZQUIERDA: ")); Serial.println(status.freeLeft ? F("LIBRE") : F("PARED"));
  Serial.print(F("DERECHA: ")); Serial.println(status.freeRight ? F("LIBRE") : F("PARED"));
}

// Freno inverso corto: invierte de inmediato el sentido actual para vencer la inercia
void reverseBrake(const int16_t speeds[4], uint32_t durationMs) {
  for (uint8_t i = 0; i < 4; i++) motorController.currentPWM[i] = speeds[i];
  motorController.setSpeeds(speeds[0], speeds[1], speeds[2], speeds[3]);
  motorController.update();
  const uint32_t t0 = millis();
  while (millis() - t0 < durationMs) motorController.update();
  motorController.stop();
}

// Avanza una celda (280 mm) con yaw lock IMU proporcional.
// Un fallo recuperable devuelve false para volver a SCAN; nunca detiene la FSM.
bool advanceOneCell() {
  if (!imuFound) {
    resetMuxChannel(MUX_CH_IMU);
    Serial.println(F("[AVANCE] IMU recuperando -> SCAN"));
    return false;
  }
  motorController.stop();
  encoderManager.resetTicks();
  odometryMm = 0.0f;
  while (Serial.available() > 0) Serial.read();

  lastImuUs = micros();
  updateYaw();
  const float targetYaw = yawDeg;  // Rumbo objetivo = rumbo al iniciar

  uint16_t dF = 0, dL = 0, dR = 0;
  Serial.print(F("\n[AVANCE] 1 celda ("));
  Serial.print(CELL_MM, 0);
  Serial.println(F(" mm), IMU yaw lock"));

  const uint32_t startMs = millis();
  uint32_t lastLogMs = 0;
  bool activeBrake = true;
  bool completed = true;
  int16_t currentBaseSpeed = NAV_BASE_PWM;
  const __FlashStringHelper* reason = F("Odometria");

  while (true) {
    const uint32_t now = millis();

    if (Serial.available() > 0) {
      const char c = Serial.read();
      if (c == 's' || c == 'S') {
        Serial.println(F("[NAV] Comando stop ignorado"));
      }
    }
    if (now - startMs > NAV_TIMEOUT_MS) {
      reason = F("TIMEOUT");
      completed = false;
      resetMuxChannel(MUX_CH_IMU);
      Serial.println(F("[AVANCE] Timeout recuperando -> SCAN"));
      break;
    }

    updateYaw();
    dF = readDistanceSensor(frontSensor, MUX_CH_FRONT, fOk);
    dL = readDistanceSensor(leftSensor, MUX_CH_LEFT, lOk);
    dR = readDistanceSensor(rightSensor, MUX_CH_RIGHT, rOk);
    recoverTofBusIfNeeded(dF, dL, dR);
    odometryMm = readOdometryMm();

    if (odometryMm >= CELL_MM) break;
    if (dF > 0 && dF <= NAV_STOP_FRONT_MM) {
      reason = F("Pared frontal");
      break;
    }

    const float yawError = wrap180(yawDeg - targetYaw);
    if (fabs(yawError) > NAV_MAX_YAW_ABORT_DEG) {
      reason = F("Rumbo fuera de limite");
      completed = false;
      resetMuxChannel(MUX_CH_IMU);
      Serial.println(F("[AVANCE] Yaw recuperando -> SCAN"));
      break;
    }

    // Rampa de PWM 200 a 80 en los ultimos 50 mm
    if (odometryMm >= NAV_DECEL_START_MM) {
      currentBaseSpeed = map(static_cast<long>(odometryMm), static_cast<long>(NAV_DECEL_START_MM),
                             static_cast<long>(CELL_MM), NAV_BASE_PWM, NAV_DECEL_END_PWM);
      currentBaseSpeed = constrain(currentBaseSpeed, NAV_DECEL_END_PWM, NAV_BASE_PWM);
    } else {
      currentBaseSpeed = NAV_BASE_PWM;
    }

    // Cada muestra fresca del gyro corrige los dos lados de forma diferencial.
    float correction = NAV_YAW_LOCK_KP * yawError;
    correction = constrain(correction, -NAV_CORR_LIMIT, NAV_CORR_LIMIT);
    const int16_t leftPwm = constrain(currentBaseSpeed - static_cast<int16_t>(correction), 0, 255);
    const int16_t rightPwm = constrain(currentBaseSpeed + static_cast<int16_t>(correction), 0, 255);
    motorController.setSpeeds(leftPwm, rightPwm, leftPwm, rightPwm);
    motorController.update();

    if (now - lastLogMs >= 100UL) {
      lastLogMs = now;
      Serial.print(F("[AVANCE] YAW:"));
      Serial.print(yawDeg - targetYaw, 1);
      Serial.print(F(" F:"));
      Serial.print(dF);
      Serial.print(F(" L:"));
      Serial.print(dL);
      Serial.print(F(" R:"));
      Serial.print(dR);
      Serial.print(F(" | "));
      Serial.print(odometryMm, 1);
      Serial.print(F("/"));
      Serial.print(CELL_MM, 0);
      Serial.println(F(" mm"));
    }
  }

  if (activeBrake) {
    const int16_t brake[4] = {-NAV_BRAKE_PWM, -NAV_BRAKE_PWM, -NAV_BRAKE_PWM, -NAV_BRAKE_PWM};
    reverseBrake(brake, NAV_BRAKE_MS);
  } else {
    motorController.stop();
  }
  odometryMm = readOdometryMm();

  Serial.print(F("[AVANCE] Finalizado por: "));
  Serial.print(reason);
  Serial.print(F(" | Distancia: "));
  Serial.print(odometryMm, 1);
  Serial.print(F(" mm | Desvio de rumbo: "));
  Serial.print(wrap180(yawDeg - targetYaw), 1);
  Serial.println(F(" grados"));

  return completed;
}

}  // namespace

Navigator::Navigator()
    : currentState(SCAN),
      plannedMove(TURN_180),
      currentHeading(NORTH),
      freeF(true),
      freeL(true),
      freeR(true),
      gyroBiasZ_(0.0f),
      turnedAngle_(0.0f),
      turnTargetDegrees_(0.0f),
      turnDirection_(1),
      turnStartedMs_(0),
      turnLastUs_(0),
      brakeStartedMs_(0),
      turnActive_(false),
      turnSlowed_(false),
      turnBraking_(false) {
  memset(&redProfile, 0, sizeof(redProfile));
  strcpy(currentColorName, "Desc");
}

void Navigator::begin() {
  plannedMove = ACTION_STOP;
  currentHeading = NORTH;
  freeF = freeL = freeR = false;
  turnActive_ = false;
  turnSlowed_ = false;
  turnBraking_ = false;

  // El sesgo se fija antes de activar la FSM para que los giros y el yaw-lock
  // compartan una sola referencia de gyro Z desde el arranque.
  calibrateGyroBias();

  // Purga: 3 lecturas preliminares de los ToF para descartar los ceros iniciales
  for (uint8_t i = 0; i < 3; i++) {
    scanDistF = readDistanceSensor(frontSensor, MUX_CH_FRONT, fOk);
    scanDistL = readDistanceSensor(leftSensor, MUX_CH_LEFT, lOk);
    scanDistR = readDistanceSensor(rightSensor, MUX_CH_RIGHT, rOk);
  }
  currentState = SCAN;  // Los perifericos ya estan listos: la FSM opera de inmediato

  // La EEPROM guarda magic + firmas; el rojo es la firma COLOR_SLOT_RED
  ColorCalibrationEeprom stored;
  EEPROM.get(0, stored);
  memset(&redProfile, 0, sizeof(redProfile));
  if (stored.magic == COLOR_CALIB_MAGIC && stored.colors[COLOR_SLOT_RED].valid == 1) {
    const ColorSignature& red = stored.colors[COLOR_SLOT_RED];
    redProfile.r = red.r;
    redProfile.g = red.g;
    redProfile.b = red.b;
    redProfile.c = red.c;
    redProfile.rn = red.rn;
    redProfile.gn = red.gn;
    redProfile.bn = red.bn;
    redProfile.tolerance = red.threshold;
    redProfile.valid = true;
  }

  Serial.print(F("[NAV] Perfil ROJO (EEPROM): "));
  if (redProfile.valid) {
    Serial.print(F("R:")); Serial.print(redProfile.r);
    Serial.print(F(" G:")); Serial.print(redProfile.g);
    Serial.print(F(" B:")); Serial.print(redProfile.b);
    Serial.print(F(" C:")); Serial.print(redProfile.c);
    Serial.print(F(" Tol:")); Serial.println(redProfile.tolerance, 3);
  } else {
    Serial.println(F("NO CALIBRADO"));
  }
  Serial.println(F("[NAV] Navegacion autonoma iniciada (clasificacion 4D COLOR_TABLE_4D)"));
}

// Captura exactamente 100 muestras separadas por 2 ms (ventana nominal de 200 ms)
// con el robot en reposo. El valor tambien alimenta updateYaw() de main.cpp para
// que avance y giro resten el mismo bias calibrado.
void Navigator::calibrateGyroBias() {
  gyroBiasZ_ = 0.0f;
  selectI2CChannel(MUX_CH_IMU);
  if (!imuFound) {
    gyroBiasZ = gyroBiasZ_;
    return;
  }

  constexpr uint8_t kSamples = 100;
  float sum = 0.0f;
  uint8_t validSamples = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (uint8_t sample = 0; sample < kSamples; ++sample) {
    selectI2CChannel(MUX_CH_IMU);
    if (readImuRaw(ax, ay, az, gx, gy, gz)) {
      sum += static_cast<float>(gz) / GYRO_LSB_PER_DPS;
      ++validSamples;
    }
    delay(2);
  }

  if (validSamples > 0) gyroBiasZ_ = sum / validSamples;
  gyroBiasZ = gyroBiasZ_;
  lastImuUs = micros();
  Serial.print(F("[NAV] Bias gyro Z (dps): "));
  Serial.println(gyroBiasZ_, 3);
}

void Navigator::startTurn180() {
  turn90(false, 180.0f);
}

void Navigator::turn90(bool turnLeft, float targetDegrees) {
  if (!imuFound) {
    // No abandona el giro sin liberar y volver a consultar el canal de la IMU.
    motorController.stop();
    delay(100);
    resetMuxChannel(MUX_CH_IMU);
    int16_t ax, ay, az, gx, gy, gz;
    if (!readImuRaw(ax, ay, az, gx, gy, gz)) {
      turnActive_ = false;
      Serial.println(F("[GIRO] IMU no recuperada"));
      return;
    }
    imuFound = true;
    lastImuUs = micros();
  }

  motorController.stop();
  selectI2CChannel(MUX_CH_IMU);
  delay(2);
  turnedAngle_ = 0.0f;
  turnTargetDegrees_ = targetDegrees;
  // +1 genera (-,+,-,+): M1/M3 atras, M2/M4 adelante (izquierda).
  // -1 genera (+,-,+,-): patron inverso para derecha.
  turnDirection_ = turnLeft ? 1 : -1;
  turnStartedMs_ = millis();
  turnLastUs_ = micros();
  turnActive_ = true;
  turnSlowed_ = false;
  turnBraking_ = false;

  const int16_t pwm = turnDirection_ * TURN_PWM;
  motorController.setSpeeds(-pwm, pwm, -pwm, pwm);
  Serial.print(F("[GIRO] Inicio "));
  Serial.print(turnLeft ? F("IZQUIERDA ") : F("DERECHA "));
  Serial.print(targetDegrees, 1);
  Serial.println(F(" grados"));
}

void Navigator::beginTurnBrake() {
  // Pulso inverso de 35 ms: replica el micro-freno activo del test validado.
  const int16_t pwm = -turnDirection_ * BRAKE_PWM;
  const int16_t brake[4] = {-pwm, pwm, -pwm, pwm};
  for (uint8_t motor = 0; motor < 4; ++motor) motorController.currentPWM[motor] = brake[motor];
  motorController.setSpeeds(brake[0], brake[1], brake[2], brake[3]);
  motorController.update();
  brakeStartedMs_ = millis();
  turnBraking_ = true;
}

Navigator::TurnResult Navigator::updateTurn() {
  if (!turnActive_) return TurnResult::FAILED;

  if (turnBraking_) {
    motorController.update();
    if (millis() - brakeStartedMs_ < BRAKE_MS) return TurnResult::IN_PROGRESS;
    motorController.stop();
    turnBraking_ = false;
    turnActive_ = false;
    lastImuUs = micros();
    Serial.print(F("[GIRO] Completado: "));
    Serial.print(turnedAngle_, 1);
    Serial.println(F(" grados"));
    return TurnResult::COMPLETED;
  }

  if (millis() - turnStartedMs_ >= TURN_TIMEOUT_MS) {
    motorController.stop();
    turnActive_ = false;
    resetMuxChannel(MUX_CH_IMU);
    Serial.println(F("[GIRO] Timeout recuperando"));
    return TurnResult::FAILED;
  }

  if (!imuFound) {
    motorController.stop();
    delay(100);
    resetMuxChannel(MUX_CH_IMU);
    int16_t ax, ay, az, gx, gy, gz;
    if (!readImuRaw(ax, ay, az, gx, gy, gz)) {
      turnActive_ = false;
      Serial.println(F("[GIRO] IMU no recuperada"));
      return TurnResult::FAILED;
    }
    imuFound = true;
    lastImuUs = micros();
    const int16_t pwm = turnDirection_ * (turnSlowed_ ? SLOW_PWM : TURN_PWM);
    motorController.setSpeeds(-pwm, pwm, -pwm, pwm);
  }

  if (Serial.available() > 0 && (Serial.peek() == 's' || Serial.peek() == 'S')) {
    Serial.read();
    Serial.println(F("[NAV] Comando stop ignorado"));
  }

  selectI2CChannel(MUX_CH_IMU);
  delayMicroseconds(300);
  int16_t ax, ay, az, gx, gy, gz;
  bool imuRead = readImuRaw(ax, ay, az, gx, gy, gz);
  if (!imuRead) {
    // Un fallo de transaccion no salta directo a SCAN: frena, deja estabilizar
    // el canal y reintenta una sola lectura limpia antes de abortar el giro.
    motorController.stop();
    delay(100);
    resetMuxChannel(MUX_CH_IMU);
    imuRead = readImuRaw(ax, ay, az, gx, gy, gz);
    if (!imuRead) {
      turnActive_ = false;
      Serial.println(F("[GIRO] I2C IMU no recuperado"));
      return TurnResult::FAILED;
    }
    const int16_t pwm = turnDirection_ * (turnSlowed_ ? SLOW_PWM : TURN_PWM);
    motorController.setSpeeds(-pwm, pwm, -pwm, pwm);
    turnLastUs_ = micros();
  }

  if (imuRead) {
    const uint32_t nowUs = micros();
    const float dt = (nowUs - turnLastUs_) * 0.000001f;
    turnLastUs_ = nowUs;
    if (dt > 0.0f && dt < 0.25f) {
      const float delta = ((static_cast<float>(gz) / GYRO_LSB_PER_DPS) - gyroBiasZ_) *
                          IMU_SCALE_FACTOR * dt;
      yawDeg = wrap180(yawDeg + delta);
      turnedAngle_ += fabs(delta);
    }
  }

  if (!turnSlowed_ && turnedAngle_ >= turnTargetDegrees_ - SLOW_ZONE_DEG) {
    turnSlowed_ = true;
    const int16_t pwm = turnDirection_ * SLOW_PWM;
    motorController.setSpeeds(-pwm, pwm, -pwm, pwm);
  }

  if (turnedAngle_ >= turnTargetDegrees_) {
    turnedAngle_ = turnTargetDegrees_;
    beginTurnBrake();
  } else {
    motorController.update();
  }
  return TurnResult::IN_PROGRESS;
}

// Vecino mas cercano 4D: evalua los 8 colores de COLOR_TABLE_4D (RGB normalizado + C/50)
// y elige la menor distancia euclidiana (aceptada si < 0.25); si no, "Desc".
String Navigator::readDetectedColor(bool& outIsGoal) {
  outIsGoal = false;
  selectI2CChannel(MUX_CH_COLOR);
  delay(2);
  color.readRaw();
  const uint16_t rawR = color.latestReading.r;
  const uint16_t rawG = color.latestReading.g;
  const uint16_t rawB = color.latestReading.b;
  const uint16_t rawC = color.latestReading.c;

  if (rawC < 5) return String("Oscuro");

  const float sum = static_cast<float>(rawR) + rawG + rawB;
  if (sum <= 0.0f) return String("Desc");
  const float nr = rawR / sum;
  const float ng = rawG / sum;
  const float nb = rawB / sum;
  const float nc = static_cast<float>(rawC) / 50.0f;

  float minDistance = 999.0f;
  int bestIndex = -1;
  for (uint8_t i = 0; i < COLOR_TABLE_4D_SIZE; i++) {
    const float dr = nr - COLOR_TABLE_4D[i].normR;
    const float dg = ng - COLOR_TABLE_4D[i].normG;
    const float db = nb - COLOR_TABLE_4D[i].normB;
    const float dc = nc - COLOR_TABLE_4D[i].normC;
    const float dist = sqrt(dr * dr + dg * dg + db * db + dc * dc);
    if (dist < minDistance) {
      minDistance = dist;
      bestIndex = i;
    }
  }

  if (bestIndex != -1 && minDistance < COLOR_MAX_DISTANCE_4D) {
    outIsGoal = COLOR_TABLE_4D[bestIndex].isGoal;
    return String(COLOR_TABLE_4D[bestIndex].name);
  }
  return String("Desc");
}

bool Navigator::isGoalColorDetected() {
  bool isGoal = false;
  const String name = readDetectedColor(isGoal);
  strncpy(currentColorName, name.c_str(), sizeof(currentColorName) - 1);
  currentColorName[sizeof(currentColorName) - 1] = '\0';
  if (isGoal) {
    Serial.print(F("[NAV] META ROJA detectada | R:")); Serial.print(color.latestReading.r);
    Serial.print(F(" G:")); Serial.print(color.latestReading.g);
    Serial.print(F(" B:")); Serial.print(color.latestReading.b);
    Serial.print(F(" C:")); Serial.println(color.latestReading.c);
  }
  return isGoal;
}

// OLED (canal 5), 4 filas: "YAW:-152 | Desc", "F:0 L:249 R:0", "F:L L:L R:L", "ODO: 1 mm"
void Navigator::renderOled() {
  if (!oledOk) return;
  char line[24];
  selectI2CChannel(MUX_CH_OLED);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  snprintf(line, sizeof(line), "YAW:%d | %.4s", static_cast<int>(yawDeg), currentColorName);
  display.setCursor(0, 0);
  display.print(line);

  snprintf(line, sizeof(line), "F:%u L:%u R:%u", scanDistF, scanDistL, scanDistR);
  display.setCursor(0, 16);
  display.print(line);

  snprintf(line, sizeof(line), "F:%c L:%c R:%c", freeF ? 'L' : 'B', freeL ? 'L' : 'B',
           freeR ? 'L' : 'B');
  display.setCursor(0, 32);
  display.print(line);

  snprintf(line, sizeof(line), "ODO: %d mm", static_cast<int>(odometryMm));
  display.setCursor(0, 48);
  display.print(line);
  display.display();
}

void Navigator::start() {
  Serial.println(F("[NAV] FSM autonoma ya activa"));
}

bool Navigator::isRunning() const {
  return currentState != STOP;
}

Orientation Navigator::getHeading() const {
  return currentHeading;
}

Move Navigator::decideRightWallRule() {
  if (freeR) return TURN_RIGHT;
  if (freeF) return FORWARD;
  if (freeL) return TURN_LEFT;
  return TURN_180;
}

void Navigator::update() {
  switch (currentState) {
    case SCAN: {
      // Color primero: la baldosa roja es la meta y detiene la FSM
      if (isGoalColorDetected()) {
        motorController.stop();
        currentState = STOP;
        plannedMove = ACTION_STOP;
        odometryMm = readOdometryMm();
        renderOled();  // Fila 1 muestra "Rojo"
        Serial.println(F("[META ROJA DETECTADA] ROBOT DETENIDO"));
        break;
      }

      CellStatus status = scanCellWalls();
      if (scanDistF == 0 && scanDistL == 0 && scanDistR == 0) {
        // Antes de que DECIDE pueda interpretar tres fallos como un callejon,
        // deja respirar el bus y exige un segundo muestreo fresco. El segundo
        // triple-cero activa la recuperacion global de Wire/MUX/OLED.
        Serial.println(F("[SCAN] ToF 0/0/0; estabilizando I2C"));
        delay(150);
        status = scanCellWalls();
      }
      freeF = status.freeFront;
      freeL = status.freeLeft;
      freeR = status.freeRight;
      printCellStatus(status);
      odometryMm = readOdometryMm();
      renderOled();
      currentState = DECIDE;
      break;
    }

    case DECIDE:
      plannedMove = decideRightWallRule();
      Serial.print(F("[NAV] Movimiento: "));
      switch (plannedMove) {
        case FORWARD: Serial.println(F("ADELANTE")); break;
        case TURN_RIGHT: Serial.println(F("GIRO DERECHA")); break;
        case TURN_LEFT: Serial.println(F("GIRO IZQUIERDA")); break;
        case TURN_180: Serial.println(F("GIRO 180")); break;
        default: Serial.println(F("STOP")); break;
      }
      if (plannedMove == ACTION_STOP) {
        // Defensa: la regla de pared nunca genera ACTION_STOP. Si apareciera,
        // se transforma en vuelta de 180 para preservar navegacion perpetua.
        plannedMove = TURN_180;
        currentState = TURN;
      } else if (plannedMove == FORWARD) {
        currentState = ADVANCE;
      } else {
        currentState = TURN;
      }
      break;

    case TURN: {
      if (!turnActive_) {
        if (plannedMove == TURN_RIGHT) turn90(false, 90.0f);
        else if (plannedMove == TURN_LEFT) turn90(true, 90.0f);
        else if (plannedMove == TURN_180) startTurn180();
        else {
          selectI2CChannel(MUX_CH_IMU);
          Serial.println(F("[NAV] Giro invalido -> SCAN"));
          currentState = SCAN;
          break;
        }
      }

      const TurnResult result = updateTurn();
      if (result == TurnResult::COMPLETED) {
        if (plannedMove == TURN_RIGHT) {
          currentHeading = static_cast<Orientation>((currentHeading + 1) % 4);
        } else if (plannedMove == TURN_LEFT) {
          currentHeading = static_cast<Orientation>((currentHeading + 3) % 4);
        } else {  // TURN_180
          currentHeading = static_cast<Orientation>((currentHeading + 2) % 4);
        }
        currentState = ADVANCE;
      } else if (result == TurnResult::FAILED) {
        resetMuxChannel(MUX_CH_IMU);
        Serial.println(F("[NAV] Giro recuperado -> SCAN"));
        currentState = SCAN;
      }
      break;
    }

    case ADVANCE:
      advanceOneCell();
      // Exito, timeout, IMU o I2C: el siguiente ciclo siempre reescanea.
      currentState = SCAN;
      break;

    case STOP:
    default:
      // STOP solo es alcanzable desde la deteccion confirmada de baldosa roja.
      break;
  }
}
