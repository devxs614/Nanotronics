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
extern uint32_t lastImuUs;
extern float odometryMm;
extern Adafruit_SSD1306 display;

float wrap180(float angle);
float updateYaw();
float readOdometryMm();
bool isWallFree(uint16_t dist);

namespace {

constexpr uint8_t NAV_CH_FRONT = 2;
constexpr uint8_t NAV_CH_LEFT = 3;
constexpr uint8_t NAV_CH_RIGHT = 4;

constexpr int16_t NAV_BASE_PWM = 200;
constexpr float NAV_DECEL_START_MM = CELL_MM - 50.0f;
constexpr int16_t NAV_DECEL_END_PWM = 80;
constexpr int16_t NAV_BRAKE_PWM = 80;
constexpr uint32_t NAV_BRAKE_MS = 30UL;
constexpr uint16_t NAV_STOP_FRONT_MM = 35;
constexpr float NAV_IMU_KP = 5.5f;
constexpr float NAV_IMU_KD = 1.5f;
constexpr float NAV_LASER_WEIGHT = 0.08f;
constexpr float NAV_CORR_LIMIT = 40.0f;
constexpr float NAV_MAX_YAW_ABORT_DEG = 30.0f;
constexpr uint32_t NAV_CONTROL_PERIOD_MS = 40UL;
constexpr uint32_t NAV_TIMEOUT_MS = 8000UL;

constexpr float TURN_TARGET_DEG = 90.0f;
constexpr float TURN_SLOW_ZONE_DEG = 15.0f;
constexpr int16_t TURN_BRAKE_PWM = 35;
constexpr uint32_t TURN_BRAKE_MS = 35UL;
constexpr uint32_t TURN_TIMEOUT_MS = 4000UL;
constexpr uint32_t TURN_SETTLE_MS = 150UL;

struct CellStatus {
  bool freeFront;
  bool freeLeft;
  bool freeRight;
};

uint16_t scanDistF = 0;
uint16_t scanDistL = 0;
uint16_t scanDistR = 0;

// Lectura de un VL53L0X en modo continuo; conserva el valor previo si no hay dato nuevo
void readNavSensor(Adafruit_VL53L0X& sensor, uint8_t channel, bool ok, uint16_t& dist) {
  if (!ok) {
    dist = 0;
    return;
  }
  selectI2CChannel(channel);
  if (!sensor.isRangeComplete()) return;
  const uint16_t range = sensor.readRangeResult();
  dist = (range > 0 && range < 2000) ? range : 0;
}

void startNavSensors() {
  if (fOk) { selectI2CChannel(NAV_CH_FRONT); frontSensor.startRangeContinuous(33); }
  if (lOk) { selectI2CChannel(NAV_CH_LEFT); leftSensor.startRangeContinuous(33); }
  if (rOk) { selectI2CChannel(NAV_CH_RIGHT); rightSensor.startRangeContinuous(33); }
}

void stopNavSensors() {
  if (fOk) { selectI2CChannel(NAV_CH_FRONT); frontSensor.stopRangeContinuous(); }
  if (lOk) { selectI2CChannel(NAV_CH_LEFT); leftSensor.stopRangeContinuous(); }
  if (rOk) { selectI2CChannel(NAV_CH_RIGHT); rightSensor.stopRangeContinuous(); }
}

uint16_t readSingleRange(Adafruit_VL53L0X& sensor, uint8_t channel, bool ok) {
  if (!ok) return 0;
  selectI2CChannel(channel);
  sensor.readRange();  // La primera lectura tras cambiar de canal puede venir vieja
  const uint16_t range = sensor.readRange();
  return (range > 0 && range < 2000) ? range : 0;
}

// Lee F (canal 2), L (canal 3) y R (canal 4). Libre solo si 150 <= dist < 2000;
// dist == 0 es un error de lectura y cuenta como PARED.
CellStatus scanCellWalls() {
  scanDistF = readSingleRange(frontSensor, NAV_CH_FRONT, fOk);
  scanDistL = readSingleRange(leftSensor, NAV_CH_LEFT, lOk);
  scanDistR = readSingleRange(rightSensor, NAV_CH_RIGHT, rOk);

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

// Avanza una celda (280 mm) con yaw lock IMU + ajuste suave por laseres.
// Devuelve false si se aborto (usuario, timeout, rumbo fuera de limite o IMU ausente).
bool advanceOneCell() {
  if (!imuFound) {
    Serial.println(F("[AVANCE] IMU no disponible, abortado"));
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
  startNavSensors();

  Serial.print(F("\n[AVANCE] 1 celda ("));
  Serial.print(CELL_MM, 0);
  Serial.println(F(" mm), IMU yaw lock + laser"));

  const uint32_t startMs = millis();
  uint32_t lastControlMs = 0;
  uint32_t lastLogMs = 0;
  float lastImuError = 0.0f;
  bool activeBrake = true;
  bool completed = true;
  int16_t currentBaseSpeed = NAV_BASE_PWM;
  const __FlashStringHelper* reason = F("Odometria");

  while (true) {
    const uint32_t now = millis();

    if (Serial.available() > 0) {
      const char c = Serial.read();
      if (c == 's' || c == 'S') {
        reason = F("Usuario");
        activeBrake = false;
        completed = false;
        break;
      }
    }
    if (now - startMs > NAV_TIMEOUT_MS) {
      reason = F("TIMEOUT");
      activeBrake = false;
      completed = false;
      break;
    }

    updateYaw();
    readNavSensor(frontSensor, NAV_CH_FRONT, fOk, dF);
    readNavSensor(leftSensor, NAV_CH_LEFT, lOk, dL);
    readNavSensor(rightSensor, NAV_CH_RIGHT, rOk, dR);
    odometryMm = readOdometryMm();

    if (odometryMm >= CELL_MM) break;
    if (dF > 0 && dF <= NAV_STOP_FRONT_MM) {
      reason = F("Pared frontal");
      break;
    }

    const float imuError = wrap180(yawDeg - targetYaw);
    if (fabs(imuError) > NAV_MAX_YAW_ABORT_DEG) {
      reason = F("Rumbo fuera de limite");
      activeBrake = false;
      completed = false;
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

    if (now - lastControlMs >= NAV_CONTROL_PERIOD_MS) {
      lastControlMs = now;

      // El laser solo ayuda con dos paredes; con un hueco lateral se ignora
      const bool wallL = (dL > 0 && dL < UMBRAL_PARED_MM);
      const bool wallR = (dR > 0 && dR < UMBRAL_PARED_MM);
      float laserError = 0.0f;
      if (wallL && wallR) laserError = (static_cast<float>(dL) - static_cast<float>(dR)) * NAV_LASER_WEIGHT;

      const float dImuError = imuError - lastImuError;
      lastImuError = imuError;

      float correction = (imuError * NAV_IMU_KP) + (dImuError * NAV_IMU_KD) + laserError;
      correction = constrain(correction, -NAV_CORR_LIMIT, NAV_CORR_LIMIT);

      const int16_t left = constrain(currentBaseSpeed - static_cast<int16_t>(correction), 0, 255);
      const int16_t right = constrain(currentBaseSpeed + static_cast<int16_t>(correction), 0, 255);
      motorController.setSpeeds(left, right, left, right);
    }
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
  stopNavSensors();
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

// Giro de 90 grados sobre su eje por acumulacion de delta IMU.
// Devuelve false si se aborto (usuario, timeout o IMU ausente).
bool turn90(bool turnLeft) {
  if (!imuFound) {
    Serial.println(F("[GIRO] IMU no disponible"));
    return false;
  }
  motorController.stop();
  while (Serial.available() > 0) Serial.read();

  // Izquierda: M1/M3 atras, M2/M4 adelante. Derecha: el inverso.
  const int16_t dir = turnLeft ? 1 : -1;
  float turnedAngle = 0.0f;
  bool slow = false;
  bool completed = true;

  Serial.print(F("\n[GIRO] "));
  Serial.println(turnLeft ? F("IZQUIERDA 90") : F("DERECHA 90"));

  lastImuUs = micros();
  const uint32_t startMs = millis();
  const int16_t basePwm = dir * TURN_BASE_PWM;
  motorController.setSpeeds(-basePwm, basePwm, -basePwm, basePwm);

  while (turnedAngle < TURN_TARGET_DEG) {
    if (millis() - startMs > TURN_TIMEOUT_MS) {
      Serial.println(F("[GIRO] TIMEOUT"));
      completed = false;
      break;
    }
    if (Serial.available() > 0 && (Serial.peek() == 's' || Serial.peek() == 'S')) {
      Serial.read();
      Serial.println(F("[GIRO] Abortado"));
      completed = false;
      break;
    }

    turnedAngle += fabs(updateYaw());

    if (!slow && turnedAngle >= (TURN_TARGET_DEG - TURN_SLOW_ZONE_DEG)) {
      slow = true;
      const int16_t slowPwm = dir * TURN_SLOW_PWM;
      motorController.setSpeeds(-slowPwm, slowPwm, -slowPwm, slowPwm);
    }
    motorController.update();
  }

  const int16_t brakePwm = -dir * TURN_BRAKE_PWM;
  const int16_t brake[4] = {-brakePwm, brakePwm, -brakePwm, brakePwm};
  reverseBrake(brake, TURN_BRAKE_MS);
  lastImuUs = micros();

  Serial.print(F("Giro completado | Ángulo acumulado: "));
  Serial.print(turnedAngle, 2);
  Serial.print(F("° | Error: "));
  Serial.print(turnedAngle - TURN_TARGET_DEG, 2);
  Serial.println(F("°"));
  return completed;
}

}  // namespace

Navigator::Navigator()
    : currentState(STOP),
      plannedMove(ACTION_STOP),
      currentHeading(NORTH),
      freeF(true),
      freeL(true),
      freeR(true) {
  memset(&redProfile, 0, sizeof(redProfile));
  strcpy(currentColorName, "Desc");
}

void Navigator::begin() {
  plannedMove = ACTION_STOP;
  currentHeading = NORTH;
  freeF = freeL = freeR = false;

  // Purga: 3 lecturas preliminares de los ToF para descartar los ceros iniciales
  for (uint8_t i = 0; i < 3; i++) {
    readSingleRange(frontSensor, NAV_CH_FRONT, fOk);
    readSingleRange(leftSensor, NAV_CH_LEFT, lOk);
    readSingleRange(rightSensor, NAV_CH_RIGHT, rOk);
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

// Vecino mas cercano 4D: evalua los 8 colores de COLOR_TABLE_4D (RGB normalizado + C/50)
// y elige la menor distancia euclidiana (aceptada si < 0.25); si no, "Desc".
String Navigator::readDetectedColor(bool& outIsGoal) {
  outIsGoal = false;
  selectI2CChannel(1);
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
  selectI2CChannel(5);
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
  if (currentState != STOP) return;
  currentHeading = NORTH;
  currentState = SCAN;
  Serial.println(F("[NAV] Navegacion iniciada (regla pared derecha)"));
}

void Navigator::stop() {
  motorController.stop();
  currentState = STOP;
  plannedMove = ACTION_STOP;
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

      const CellStatus status = scanCellWalls();
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
        currentState = STOP;
      } else if (plannedMove == FORWARD) {
        currentState = ADVANCE;
      } else {
        currentState = TURN;
      }
      break;

    case TURN: {
      bool ok = true;
      if (plannedMove == TURN_RIGHT) {
        ok = turn90(false);
        if (ok) currentHeading = static_cast<Orientation>((currentHeading + 1) % 4);
      } else if (plannedMove == TURN_LEFT) {
        ok = turn90(true);
        if (ok) currentHeading = static_cast<Orientation>((currentHeading + 3) % 4);
      } else if (plannedMove == TURN_180) {
        ok = turn90(false);
        if (ok) {
          delay(TURN_SETTLE_MS);
          ok = turn90(false);
        }
        if (ok) currentHeading = static_cast<Orientation>((currentHeading + 2) % 4);
      }
      if (ok) {
        delay(TURN_SETTLE_MS);
        currentState = ADVANCE;
      } else {
        stop();
      }
      break;
    }

    case ADVANCE:
      if (advanceOneCell()) {
        delay(TURN_SETTLE_MS);
        currentState = SCAN;
      } else {
        stop();
      }
      break;

    case STOP:
    default:
      motorController.stop();
      break;
  }
}
