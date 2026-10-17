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
extern bool frontOk;
extern bool rightOk;
extern bool leftOk;
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

constexpr int16_t NAV_DECEL_END_PWM = 50;
constexpr int16_t NAV_BRAKE_PWM = 60;
constexpr uint32_t NAV_BRAKE_MS = 35UL;
constexpr uint16_t NAV_STOP_FRONT_MM = 35;
constexpr float NAV_MAX_YAW_ABORT_DEG = 30.0f;
constexpr uint32_t NAV_TIMEOUT_MS = 8000UL;

// Parametros validados en el test de giros IMU.
constexpr int16_t TURN_PWM = 40;
constexpr int16_t SLOW_PWM = 22;
constexpr int16_t BRAKE_PWM = 45;
constexpr float SLOW_ZONE_DEG = 15.0f;
constexpr uint32_t BRAKE_MS = 35UL;
constexpr uint32_t TURN_SETTLE_MS = 100UL;
constexpr uint32_t TURN_TIMEOUT_MS = 6000UL;
constexpr uint8_t TOF_I2C_ADDRESS = 0x29;
constexpr uint16_t TOF_MIN_VALID_MM = 1;
constexpr uint16_t TOF_MAX_VALID_MM = 2000;
constexpr uint32_t TOF_RESPONSE_TIMEOUT_MS = 100UL;

struct CellStatus {
  bool freeFront;
  bool freeLeft;
  bool freeRight;
};

const char* trackColorName(TrackColor colorId) {
  switch (colorId) {
    case COLOR_CYAN: return "CIAN";
    case COLOR_GREEN: return "VERDE";
    case COLOR_YELLOW: return "AMARILLO";
    case COLOR_PINK: return "ROSA";
    case COLOR_ORANGE: return "NARANJA";
    case COLOR_RED: return "ROJO";
    case COLOR_LIME: return "VERDE LIMA";
    case COLOR_FLOOR:
    default: return "PISO";
  }
}

uint16_t scanDistF = 0;
uint16_t scanDistL = 0;
uint16_t scanDistR = 0;
bool tofSensorsInitialized = false;
float targetYaw = 0.0f;

void resetMuxChannel(uint8_t channel) {
  selectI2CChannel(channel);
}

bool initializeTofOnce(Adafruit_VL53L0X& sensor, uint8_t channel) {
  selectI2CChannel(channel);
  // El driver Adafruit no implementa setTimeout(); su declaracion esta
  // deshabilitada. Wire watchdog + el limite de 100 ms de la lectura protegen
  // el bus sin reinicializar el VL53L0X en el ciclo normal de navegacion.
  const bool initialized = sensor.begin(TOF_I2C_ADDRESS, false, &Wire,
                                        Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
  delayMicroseconds(200);
  return initialized;
}

void initializeTofSensors() {
  if (tofSensorsInitialized) return;

  // Inicializacion unica: cada VL53L0X comparte 0x29, pero esta aislado por
  // su canal TCA9548A (F=2, L=4, R=3).
  fOk = initializeTofOnce(frontSensor, MUX_CH_FRONT);
  lOk = initializeTofOnce(leftSensor, MUX_CH_LEFT);
  rOk = initializeTofOnce(rightSensor, MUX_CH_RIGHT);
  frontOk = fOk;
  leftOk = lOk;
  rightOk = rOk;
  tofSensorsInitialized = true;

  Serial.print(F("[TOF] Inicio unico F/L/R: "));
  Serial.print(fOk ? F("OK") : F("FALLO"));
  Serial.print('/');
  Serial.print(lOk ? F("OK") : F("FALLO"));
  Serial.print('/');
  Serial.println(rOk ? F("OK") : F("FALLO"));
}

// Cada llamada toma una conversion fisica individual. Nunca devuelve un dato
// anterior: una muestra ausente, invalida, fuera de rango o con timeout es 0.
uint16_t readDistanceSensor(Adafruit_VL53L0X& sensor, uint8_t channel, bool ok) {
  if (!ok) return 0;

  const auto takeFreshMeasurement = [&]() -> uint16_t {
    selectI2CChannel(channel);
    const uint32_t startedMs = millis();
    const uint16_t range = sensor.readRange();
    const uint8_t status = sensor.readRangeStatus();
    const bool timedOut = sensor.timeoutOccurred() ||
                          (millis() - startedMs > TOF_RESPONSE_TIMEOUT_MS);
    return (!timedOut && range >= TOF_MIN_VALID_MM && range <= TOF_MAX_VALID_MM && status == 0)
               ? range
               : 0;
  };
  return takeFreshMeasurement();
}

// Lee F (canal 2), L (canal 4) y R (canal 3). Un 0 representa retorno
// ausente/fuera de rango y es LIBRE; solo 1..179 mm representa PARED.
CellStatus scanCellWalls() {
  scanDistF = readDistanceSensor(frontSensor, MUX_CH_FRONT, fOk);
  scanDistL = readDistanceSensor(leftSensor, MUX_CH_LEFT, lOk);
  scanDistR = readDistanceSensor(rightSensor, MUX_CH_RIGHT, rOk);

  CellStatus status;
  status.freeFront = (scanDistF == 0 || scanDistF >= UMBRAL_PARED_MM);
  status.freeLeft = (scanDistL == 0 || scanDistL >= UMBRAL_PARED_MM);
  status.freeRight = (scanDistR == 0 || scanDistR >= UMBRAL_PARED_MM);
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

// Avanza una celda con el PID recto calibrado (Kp=4.0, Kd=0.0).
// Un fallo recuperable devuelve false para volver a SCAN; nunca detiene la FSM.
bool advanceOneCell(float targetDistanceMm = CELL_MM) {
  if (!imuFound) {
    resetMuxChannel(MUX_CH_IMU);
    Serial.println(F("[AVANCE] IMU recuperando -> SCAN"));
    return false;
  }
  motorController.stop();
  while (Serial.available() > 0) Serial.read();

  // El chasis queda inmovil antes de congelar el rumbo de referencia de esta
  // casilla. Asi el PID no hereda inercia del movimiento anterior.
  delay(100);
  lastImuUs = micros();
  updateYaw();
  targetYaw = yawDeg;  // Rumbo objetivo = rumbo al iniciar
  encoderManager.resetTicks();
  odometryMm = 0.0f;
  float lastProgressMm = odometryMm;
  uint32_t lastProgressMs = millis();
  float lastYawError = 0.0f;
  uint32_t lastControlUs = micros();
  const float decelStartMm = targetDistanceMm - 50.0f;

  uint16_t dF = 0, dL = 0, dR = 0;
  Serial.print(F("\n[AVANCE] 1 celda ("));
  Serial.print(targetDistanceMm, 0);
  Serial.println(F(" mm), IMU yaw lock"));

  const uint32_t startMs = millis();
  uint32_t lastLogMs = 0;
  bool activeBrake = true;
  bool completed = true;
  int16_t currentBaseSpeed = NAV_STRAIGHT_BASE_PWM;
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
    odometryMm = readOdometryMm();

    if (odometryMm >= targetDistanceMm) break;
    if (odometryMm - lastProgressMm >= NAV_STUCK_MIN_PROGRESS_MM) {
      lastProgressMm = odometryMm;
      lastProgressMs = now;
    } else if (now - lastProgressMs > NAV_STUCK_PROGRESS_TIMEOUT_MS) {
      reason = F("ATASCADO");
      completed = false;
      activeBrake = false;
      navigator.triggerStuckRecovery();
      break;
    }
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

    // Rampa de PWM 100 a 50 desde los 230 mm, manteniendo el yaw lock.
    if (odometryMm >= decelStartMm) {
      currentBaseSpeed = map(static_cast<long>(odometryMm), static_cast<long>(decelStartMm),
                             static_cast<long>(targetDistanceMm), NAV_STRAIGHT_BASE_PWM, NAV_DECEL_END_PWM);
      currentBaseSpeed = constrain(currentBaseSpeed, NAV_DECEL_END_PWM, NAV_STRAIGHT_BASE_PWM);
    } else {
      currentBaseSpeed = NAV_STRAIGHT_BASE_PWM;
    }

    // Signo validado en Test 15: yaw positivo suma a la izquierda y resta a
    // la derecha. Kd queda en 0.0, pero se conserva la forma PD calibrable.
    const uint32_t nowUs = micros();
    const float controlDt = (nowUs - lastControlUs) * 0.000001f;
    lastControlUs = nowUs;
    const float derivative = controlDt > 0.0005f
                                 ? (yawError - lastYawError) / controlDt
                                 : 0.0f;
    const float correction = KP_STRAIGHT * yawError + KD_STRAIGHT * derivative;
    const int16_t leftPwm = constrain(currentBaseSpeed + static_cast<int16_t>(correction), 40, 160);
    const int16_t rightPwm = constrain(currentBaseSpeed - static_cast<int16_t>(correction), 40, 160);
    motorController.setSpeeds(leftPwm, rightPwm, leftPwm, rightPwm);
    motorController.update();
    lastYawError = yawError;

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
      Serial.print(targetDistanceMm, 0);
      Serial.println(F(" mm"));
    }
  }

  if (activeBrake) {
    const int16_t brake[4] = {-NAV_BRAKE_PWM, -NAV_BRAKE_PWM, -NAV_BRAKE_PWM, -NAV_BRAKE_PWM};
    reverseBrake(brake, NAV_BRAKE_MS);
  } else {
    motorController.stop();
  }

  // La correccion de rumbo ocurre antes de la pausa y del nuevo escaneo. El
  // centrado longitudinal solo se aplica cuando se alcanzo la nueva casilla.
  navigator.snapYawToNearest90();
  odometryMm = readOdometryMm();
  if (odometryMm >= targetDistanceMm) navigator.alignWithFrontWall();
  // Los 3000 ms reglamentarios se cumplen con motores detenidos mientras ToF,
  // OLED y la telemetria Serie continúan actualizandose.
  navigator.restWithTelemetry(NAV_CELL_REST_MS);
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
      colorCalibrationLoaded_(false),
      currentTrackColor_(COLOR_FLOOR),
      gyroBiasZ_(0.0f),
      turnedAngle_(0.0f),
      turnTargetDegrees_(0.0f),
      turnDirection_(1),
      turnStartedMs_(0),
      turnLastUs_(0),
      brakeStartedMs_(0),
      settleStartedMs_(0),
      restStartedMs_(0),
      lastRestTelemetryMs_(0),
      turnActive_(false),
      turnSlowed_(false),
      turnBraking_(false),
      turnSettling_(false),
      turnResting_(false),
      turn180Sequence_(false),
      turn180SecondTurn_(false),
      turn180InterRest_(false),
      turn180FinalRest_(false),
      turn180PauseStartedMs_(0) {
  memset(&redProfile, 0, sizeof(redProfile));
  memset(&colorCalibration_, 0, sizeof(colorCalibration_));
  strcpy(currentColorName, "PISO");
}

void Navigator::begin() {
  plannedMove = ACTION_STOP;
  currentHeading = NORTH;
  freeF = freeL = freeR = false;
  turnActive_ = false;
  turnSlowed_ = false;
  turnBraking_ = false;
  turnSettling_ = false;
  turnResting_ = false;
  turn180Sequence_ = false;
  turn180SecondTurn_ = false;
  turn180InterRest_ = false;
  turn180FinalRest_ = false;
  foundCyan = false;
  foundGreen = false;
  foundYellow = false;
  foundPink = false;
  foundOrange = false;
  foundRed = false;
  sameActionCount = 0;
  lastAction = 0;
  forceAlternativeTurn = false;
  currentTrackColor_ = COLOR_FLOOR;
  strcpy(currentColorName, "PISO");

  // Los ToF se inicializan aqui, una sola vez, antes de la primera lectura de
  // SCAN. readDistanceSensor() nunca ejecuta begin()/init().
  initializeTofSensors();

  // El sesgo se fija antes de activar la FSM para que los giros y el yaw-lock
  // compartan una sola referencia de gyro Z desde el arranque.
  calibrateGyroBias();

  // Purga: 3 lecturas preliminares de los ToF para descartar los ceros iniciales
  for (uint8_t i = 0; i < 3; i++) {
    scanDistF = readDistanceSensor(frontSensor, MUX_CH_FRONT, fOk);
    scanDistL = readDistanceSensor(leftSensor, MUX_CH_LEFT, lOk);
    scanDistR = readDistanceSensor(rightSensor, MUX_CH_RIGHT, rOk);
  }
  // Carga el bloque completo producido por el test de calibracion RGB. Las
  // firmas y sus umbrales son la fuente primaria de clasificacion si existen.
  EEPROM.get(0, colorCalibration_);
  colorCalibrationLoaded_ = colorCalibration_.magic == COLOR_CALIB_MAGIC;
  memset(&redProfile, 0, sizeof(redProfile));
  if (colorCalibrationLoaded_ && colorCalibration_.colors[COLOR_SLOT_RED].valid == 1) {
    const ColorSignature& red = colorCalibration_.colors[COLOR_SLOT_RED];
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
  // Cambia inmediatamente desde la pantalla de inicio a las cuatro filas de
  // telemetria y permanece inmovil cuatro segundos tomando muestras nuevas.
  const CellStatus startupStatus = scanCellWalls();
  freeF = startupStatus.freeFront;
  freeL = startupStatus.freeLeft;
  freeR = startupStatus.freeRight;
  odometryMm = readOdometryMm();
  renderOled();
  Serial.println(F("[NAV] Estabilizacion telemetria inicial: 4 s"));
  restWithTelemetry(NAV_STARTUP_STABILIZE_MS);

  // El primer desplazamiento no depende de la regla de pared: siempre intenta
  // recorrer la primera celda de 280 mm antes de iniciar SCAN/DECIDE normal.
  Serial.println(F("[NAV] Primer avance obligatorio: 280 mm"));
  currentState = ADVANCE;
  advanceOneCell();
  currentState = SCAN;
  Serial.println(F("[NAV] Navegacion autonoma iniciada (clasificacion RGB calibrada)"));
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
  // Dos giros izquierdos de 90° permiten frenar, reencuadrar el yaw y leer
  // ToF a mitad de maniobra, en lugar de mantener un giro ciego de 180°.
  turn180Sequence_ = true;
  turn180SecondTurn_ = false;
  turn180InterRest_ = false;
  turn180FinalRest_ = false;
  turn90(true, 90.0f);
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
      turn180Sequence_ = false;
      turn180SecondTurn_ = false;
      turn180InterRest_ = false;
      turn180FinalRest_ = false;
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
  turnSettling_ = false;
  turnResting_ = false;
  lastRestTelemetryMs_ = 0;

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
  if (!turnActive_) {
    turn180Sequence_ = false;
    turn180SecondTurn_ = false;
    turn180InterRest_ = false;
    turn180FinalRest_ = false;
    return TurnResult::FAILED;
  }

  if (turnBraking_) {
    motorController.update();
    if (millis() - brakeStartedMs_ < BRAKE_MS) return TurnResult::IN_PROGRESS;
    motorController.stop();
    turnBraking_ = false;
    turnSettling_ = true;
    settleStartedMs_ = millis();
    return TurnResult::IN_PROGRESS;
  }

  if (turnSettling_) {
    if (millis() - settleStartedMs_ < TURN_SETTLE_MS) return TurnResult::IN_PROGRESS;
    turnSettling_ = false;
    // Cada tramo de 90° termina con referencia ortogonal y, si existe una
    // pared medible al frente, con su ajuste longitudinal por ToF.
    snapYawToNearest90();
    alignWithFrontWall();

    if (turn180Sequence_ && !turn180SecondTurn_) {
      // Tras el primer cuarto de vuelta, toma muestras ToF inmovil durante
      // 300 ms antes de iniciar el segundo giro independiente de 90°.
      turn180InterRest_ = true;
      turn180PauseStartedMs_ = millis();
      lastRestTelemetryMs_ = 0;
      return TurnResult::IN_PROGRESS;
    }
    if (turn180Sequence_ && turn180SecondTurn_) {
      // El segundo tramo ya termino: conserva 150 ms de reposo antes del
      // snap final y de la pausa normal de celda.
      turn180FinalRest_ = true;
      turn180PauseStartedMs_ = millis();
      lastRestTelemetryMs_ = 0;
      return TurnResult::IN_PROGRESS;
    }

    turnResting_ = true;
    restStartedMs_ = millis();
    lastRestTelemetryMs_ = 0;
    return TurnResult::IN_PROGRESS;
  }

  if (turn180InterRest_) {
    if (lastRestTelemetryMs_ == 0 ||
        millis() - lastRestTelemetryMs_ >= NAV_REST_TELEMETRY_MS) {
      refreshStationaryTelemetry();
      lastRestTelemetryMs_ = millis();
    }
    if (millis() - turn180PauseStartedMs_ < NAV_180_INTER_TURN_REST_MS) {
      return TurnResult::IN_PROGRESS;
    }

    turn180InterRest_ = false;
    turn180SecondTurn_ = true;
    turn90(true, 90.0f);
    return TurnResult::IN_PROGRESS;
  }

  if (turn180FinalRest_) {
    if (lastRestTelemetryMs_ == 0 ||
        millis() - lastRestTelemetryMs_ >= NAV_REST_TELEMETRY_MS) {
      refreshStationaryTelemetry();
      lastRestTelemetryMs_ = millis();
    }
    if (millis() - turn180PauseStartedMs_ < NAV_180_FINAL_SNAP_REST_MS) {
      return TurnResult::IN_PROGRESS;
    }

    // El snap tras los 150 ms absorbe cualquier relajacion mecanica final.
    snapYawToNearest90();
    turn180FinalRest_ = false;
    turn180Sequence_ = false;
    turn180SecondTurn_ = false;
    turnResting_ = true;
    restStartedMs_ = millis();
    lastRestTelemetryMs_ = 0;
    return TurnResult::IN_PROGRESS;
  }

  if (turnResting_) {
    if (lastRestTelemetryMs_ == 0 ||
        millis() - lastRestTelemetryMs_ >= NAV_REST_TELEMETRY_MS) {
      refreshStationaryTelemetry();
      lastRestTelemetryMs_ = millis();
    }
    if (millis() - restStartedMs_ < NAV_CELL_REST_MS) return TurnResult::IN_PROGRESS;
    turnResting_ = false;
    turnActive_ = false;
    turn180Sequence_ = false;
    turn180SecondTurn_ = false;
    turn180InterRest_ = false;
    turn180FinalRest_ = false;
    lastImuUs = micros();
    Serial.print(F("[GIRO] Completado: "));
    Serial.print(turnedAngle_, 1);
    Serial.println(F(" grados"));
    return TurnResult::COMPLETED;
  }

  if (millis() - turnStartedMs_ >= TURN_TIMEOUT_MS) {
    motorController.stop();
    turnActive_ = false;
    turn180Sequence_ = false;
    turn180SecondTurn_ = false;
    turn180InterRest_ = false;
    turn180FinalRest_ = false;
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
      turn180Sequence_ = false;
      turn180SecondTurn_ = false;
      turn180InterRest_ = false;
      turn180FinalRest_ = false;
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
      turn180Sequence_ = false;
      turn180SecondTurn_ = false;
      turn180InterRest_ = false;
      turn180FinalRest_ = false;
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

void Navigator::snapYawToNearest90() {
  const float nearest90 = roundf(yawDeg / 90.0f) * 90.0f;
  yawDeg = nearest90;
  targetYaw = nearest90;
  // Impide que la siguiente integracion incluya el intervalo anterior al snap.
  lastImuUs = micros();
}

void Navigator::alignWithFrontWall() {
  motorController.stop();

  scanDistF = readDistanceSensor(frontSensor, MUX_CH_FRONT, fOk);
  freeF = (scanDistF == 0 || scanDistF >= UMBRAL_PARED_MM);
  const bool frontWall = scanDistF >= NAV_FRONT_ALIGN_MIN_MM &&
                         scanDistF <= NAV_FRONT_ALIGN_MAX_MM;
  if (frontWall) {
    const bool moveForward = scanDistF > NAV_FRONT_ALIGN_FORWARD_MIN_MM;
    const bool moveBackward = scanDistF < NAV_FRONT_ALIGN_REVERSE_MAX_MM;
    const uint32_t startedMs = millis();

    while ((moveForward || moveBackward) &&
           millis() - startedMs < NAV_FRONT_ALIGN_TIMEOUT_MS) {
      // Una lectura 0 significa que no hay una pared medible para usar como
      // referencia; se abandona el ajuste sin reutilizar una muestra antigua.
      scanDistF = readDistanceSensor(frontSensor, MUX_CH_FRONT, fOk);
      freeF = (scanDistF == 0 || scanDistF >= UMBRAL_PARED_MM);
      if (scanDistF == 0) break;
      if ((moveForward && scanDistF <= NAV_FRONT_ALIGN_TARGET_MM) ||
          (moveBackward && scanDistF >= NAV_FRONT_ALIGN_TARGET_MM)) break;

      const int16_t pwm = moveForward ? NAV_FRONT_ALIGN_PWM : -NAV_FRONT_ALIGN_PWM;
      motorController.setSpeeds(pwm, pwm, pwm, pwm);
      motorController.update();
      yield();
    }
  }

  motorController.stop();
  motorController.update();
  delay(NAV_FRONT_ALIGN_SETTLE_MS);
}

void Navigator::triggerStuckRecovery() {
  motorController.stop();
  motorController.update();
  Serial.println(F("[ATASCO] Sin avance; micro-reversa de 50 mm"));

  // readOdometryMm() mide la magnitud de los ticks; por ello la diferencia
  // acumulada durante la marcha atras representa correctamente los 50 mm.
  const float startedMm = readOdometryMm();
  const uint32_t startedMs = millis();
  motorController.setSpeeds(-NAV_STUCK_REVERSE_PWM, -NAV_STUCK_REVERSE_PWM,
                            -NAV_STUCK_REVERSE_PWM, -NAV_STUCK_REVERSE_PWM);
  while (readOdometryMm() - startedMm < NAV_STUCK_REVERSE_MM &&
         millis() - startedMs < NAV_STUCK_REVERSE_TIMEOUT_MS) {
    motorController.update();
    yield();
  }

  motorController.stop();
  motorController.update();
  snapYawToNearest90();
  delay(NAV_STUCK_SETTLE_MS);
}

// Mantiene datos frescos mientras el robot espera inmovil entre casillas.
// No reinicializa sensores ni reutiliza mediciones anteriores.
void Navigator::refreshStationaryTelemetry() {
  motorController.stop();
  updateYaw();
  const CellStatus status = scanCellWalls();
  freeF = status.freeFront;
  freeL = status.freeLeft;
  freeR = status.freeRight;
  odometryMm = readOdometryMm();
  renderOled();

  Serial.print(F("[PAUSA] YAW:"));
  Serial.print(yawDeg, 1);
  Serial.print(F(" F:"));
  Serial.print(scanDistF);
  Serial.print(F(" L:"));
  Serial.print(scanDistL);
  Serial.print(F(" R:"));
  Serial.println(scanDistR);
}

void Navigator::restWithTelemetry(uint32_t durationMs) {
  motorController.stop();
  const uint32_t startedMs = millis();
  uint32_t lastTelemetryMs = 0;

  while (millis() - startedMs < durationMs) {
    const uint32_t nowMs = millis();
    if (lastTelemetryMs == 0 || nowMs - lastTelemetryMs >= NAV_REST_TELEMETRY_MS) {
      refreshStationaryTelemetry();
      lastTelemetryMs = millis();
    }
    yield();
  }
  motorController.stop();
}

// Clasificacion 1-NN por menor distancia euclidiana RGB contra los ocho
// centroides de pista. Solo una muestra sin senal se considera Piso de entrada.
TrackColor Navigator::readDetectedColor() {
  selectI2CChannel(MUX_CH_COLOR);
  delay(2);
  color.readRaw();
  const uint16_t rawR = color.latestReading.r;
  const uint16_t rawG = color.latestReading.g;
  const uint16_t rawB = color.latestReading.b;
  const uint16_t rawC = color.latestReading.c;

  const float sum = static_cast<float>(rawR) + rawG + rawB;
  if (sum < 5.0f || rawC < 3) {
    Serial.println(F("[COLOR] PISO | sin señal"));
    return COLOR_FLOOR;
  }

  const float nr = rawR / sum;
  const float ng = rawG / sum;
  const float nb = rawB / sum;
  float minDistance = 999.0f;
  uint8_t bestIndex = 0;
  for (uint8_t i = 0; i < COLOR_CENTROID_COUNT; i++) {
    const float dr = nr - COLOR_CENTROIDS[i].rNorm;
    const float dg = ng - COLOR_CENTROIDS[i].gNorm;
    const float db = nb - COLOR_CENTROIDS[i].bNorm;
    const float dist = sqrt(dr * dr + dg * dg + db * db);
    if (dist < minDistance) {
      minDistance = dist;
      bestIndex = i;
    }
  }

  const TrackColor detected = COLOR_CENTROIDS[bestIndex].color;
  Serial.print(F("[COLOR] "));
  Serial.print(trackColorName(detected));
  Serial.print(F(" | d_min: "));
  Serial.println(minDistance, 4);
  return detected;
}

bool Navigator::isGoalColorDetected() {
  currentTrackColor_ = readDetectedColor();
  strncpy(currentColorName, trackColorName(currentTrackColor_), sizeof(currentColorName) - 1);
  currentColorName[sizeof(currentColorName) - 1] = '\0';

  switch (currentTrackColor_) {
    case COLOR_CYAN: foundCyan = true; break;
    case COLOR_GREEN: foundGreen = true; break;
    case COLOR_YELLOW: foundYellow = true; break;
    case COLOR_PINK: foundPink = true; break;
    case COLOR_ORANGE: foundOrange = true; break;
    case COLOR_RED:
      foundRed = true;
      if (allRequiredColorsFound()) {
        Serial.println(F("[META] Pista completada: colores requeridos + ROJO."));
        return true;
      }
      Serial.println(F("[COLOR] ROJO detectado; faltan colores. Navegacion continua."));
      break;
    case COLOR_LIME:
      // Verde Lima es una referencia visual valida, no parte del checklist.
      break;
    case COLOR_FLOOR:
    default:
      // Piso se muestra en OLED, pero es neutro para el checklist.
      break;
  }
  return false;
}

bool Navigator::allRequiredColorsFound() const {
  return foundCyan && foundGreen && foundYellow && foundPink && foundOrange;
}

// OLED (canal 5), 4 filas: "YAW:-152 | Desc", "F:0 L:249 R:0", "F:L L:L R:L", "ODO: 1 mm"
void Navigator::renderOled() {
  if (!oledOk) return;
  char line[24];
  selectI2CChannel(MUX_CH_OLED);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  char yawText[8];
  dtostrf(yawDeg, 0, 1, yawText);
  snprintf(line, sizeof(line), "YAW:%s | %s", yawText, currentColorName);
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
  // Regla base de pared derecha: derecha -> frente -> izquierda -> retorno.
  Move selectedMove = TURN_180;
  if (freeR) selectedMove = TURN_RIGHT;
  else if (freeF) selectedMove = FORWARD;
  else if (freeL) selectedMove = TURN_LEFT;

  // Tras detectar cuatro acciones repetidas, el siguiente DECIDE prioriza un
  // movimiento distinto del ultimo para abandonar el patron circular.
  if (forceAlternativeTurn) {
    forceAlternativeTurn = false;
    if (lastAction == 2 && freeL) {
      selectedMove = TURN_LEFT;
    } else if (lastAction == 3 && freeR) {
      selectedMove = TURN_RIGHT;
    } else if (freeF) {
      selectedMove = FORWARD;
    } else if (freeR) {
      selectedMove = TURN_RIGHT;
    } else if (freeL) {
      selectedMove = TURN_LEFT;
    }
    Serial.println(F("[ANTI-LOOP] Ejecutando accion alternativa."));
  }

  uint8_t action = 0;
  if (selectedMove == FORWARD) action = 1;
  else if (selectedMove == TURN_RIGHT) action = 2;
  else if (selectedMove == TURN_LEFT) action = 3;

  if (action == 0) {
    sameActionCount = 0;
    lastAction = 0;
    return selectedMove;
  }

  if (action == lastAction) ++sameActionCount;
  else {
    sameActionCount = 1;
    lastAction = action;
  }

  if (sameActionCount >= NAV_ANTI_LOOP_ACTION_LIMIT) {
    Serial.println(F("[ANTI-LOOP] Repeticion de 4 acciones detectada."));
    sameActionCount = 0;

    // Cuatro avances iguales se rompen de inmediato girando hacia la primera
    // apertura lateral disponible, con preferencia derecha de la regla base.
    if (action == 1 && (freeR || freeL)) {
      selectedMove = freeR ? TURN_RIGHT : TURN_LEFT;
      lastAction = selectedMove == TURN_RIGHT ? 2 : 3;
      sameActionCount = 1;
      Serial.println(F("[ANTI-LOOP] Avances repetidos; giro lateral forzado."));
    } else {
      forceAlternativeTurn = true;
    }
  }

  return selectedMove;
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
        // Un giro ya completó 100 ms de asentamiento y 3 s inmóvil; ahora
        // adquiere un SCAN fresco antes de calcular otra decisión de pared.
        currentState = SCAN;
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
