#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"

// Formato que guarda tests_2.0/09_test_color_calibration_eeprom.cpp en la EEPROM (direccion 0):
// codigo magic + una firma por color (Cian, Amarillo, Naranja, Rosa, Rojo, Verde, Piso).
struct ColorSignature {
  uint16_t r, g, b, c;   // Promedio crudo
  float rn, gn, bn;      // RGB normalizado
  float threshold;       // Distancia maxima aceptada (espacio normalizado)
  uint8_t valid;
};

constexpr uint8_t COLOR_SLOT_COUNT = 7;
constexpr uint8_t COLOR_SLOT_CYAN = 0;
constexpr uint8_t COLOR_SLOT_YELLOW = 1;
constexpr uint8_t COLOR_SLOT_ORANGE = 2;
constexpr uint8_t COLOR_SLOT_PINK = 3;
constexpr uint8_t COLOR_SLOT_RED = 4;
constexpr uint8_t COLOR_SLOT_GREEN = 5;
constexpr uint8_t COLOR_SLOT_FLOOR = 6;
constexpr uint32_t COLOR_CALIB_MAGIC = 0x434C5233UL;  // "CLR3"

struct ColorCalibrationEeprom {
  uint32_t magic;
  ColorSignature colors[COLOR_SLOT_COUNT];
};

// Perfil del color de meta (rojo) tomado de la calibracion
struct ColorProfile {
  uint16_t r, g, b, c;
  float rn, gn, bn;
  float tolerance;
  bool valid;
};

enum State { SCAN, DECIDE, TURN, ADVANCE, STOP };
enum Move { FORWARD, TURN_RIGHT, TURN_LEFT, TURN_180, ACTION_STOP };
enum Orientation { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

class Navigator {
 private:
  enum class TurnResult : uint8_t { IN_PROGRESS, COMPLETED, FAILED };
  enum class TrackBState : uint8_t {
    SECTION1_EXPLORE,
    SECTION1_GRAB,
    TURNING,
    SECTION1_ADVANCE,
    SECTION2_SCAN,
    SECTION2_ADVANCE,
    SECTION3_SCAN,
    SECTION3_ADVANCE,
    FINISHED
  };

  State currentState;
  Move plannedMove;
  Orientation currentHeading;
  TrackMode activeMode = ACTIVE_TRACK_MODE;
  TrackBState trackBState_ = TrackBState::SECTION1_EXPLORE;
  TrackBState trackBAfterTurn_ = TrackBState::SECTION1_EXPLORE;
  Move trackBMove_ = FORWARD;
  bool hasBall = false;
  TrackBSection sec1State = SEC1_SEARCH_BALL;
  bool trackBLeavingContainer_ = false;
  bool startupModeDisplay_ = true;
  uint16_t trackBScore_ = 0;
  uint16_t qtrPeak_ = 0;
  uint32_t trackBLastScanMs_ = 0;

  // Banderas de lectura de pared
  bool freeF, freeL, freeR;

  ColorProfile redProfile;  // Color de meta cargado de la EEPROM
  ColorCalibrationEeprom colorCalibration_;
  bool colorCalibrationLoaded_;
  char currentColorName[12];  // Ultimo color clasificado (Rojo, Cian, ..., Desc)
  TrackColor currentTrackColor_;

  // Checklist persistente durante la navegacion actual. Piso no participa.
  bool foundCyan = false;
  bool foundGreen = false;
  bool foundYellow = false;
  bool foundPink = false;
  bool foundOrange = false;
  bool foundRed = false;

  // Detector de repeticion de acciones: 1=frente, 2=derecha, 3=izquierda.
  uint8_t sameActionCount = 0;
  uint8_t lastAction = 0;
  bool forceAlternativeTurn = false;

  // Estado del giro no bloqueante, integrado directamente desde el gyro Z.
  float gyroBiasZ_;
  float turnedAngle_;
  float turnTargetDegrees_;
  int8_t turnDirection_;
  uint32_t turnStartedMs_;
  uint32_t turnLastUs_;
  uint32_t brakeStartedMs_;
  uint32_t settleStartedMs_;
  uint32_t restStartedMs_;
  uint32_t lastRestTelemetryMs_;
  bool turnActive_;
  bool turnSlowed_;
  bool turnBraking_;
  bool turnSettling_;
  bool turnResting_;
  bool turn180Sequence_;
  bool turn180SecondTurn_;
  bool turn180InterRest_;
  bool turn180FinalRest_;
  uint32_t turn180PauseStartedMs_;

  Move decideRightWallRule();
  TrackColor readDetectedColor();  // Canal 1: clasificador RGB 1-NN sobre COLOR_CENTROIDS
  void calibrateGyroBias();
  void turn90(bool turnLeft, float targetDegrees = 90.0f);
  void startTurn180();
  TurnResult updateTurn();
  void beginTurnBrake();
  void refreshStationaryTelemetry();
  void checkAndRecoverFrontZero();
  void updateTrackB();
  void startTrackBTurn(Move move, TrackBState nextState);
  void updateTrackBTurn();
  void captureTrackBBall();
  bool advanceTrackBSection2();
  bool isTrackBWhiteLine();
  void avoidTrackBWhiteLine();
  void updateTrackBColor();
  uint8_t trackBSectionNumber() const;

 public:
  Navigator();
  void begin();
  void start();                // Compatibilidad Serial; la FSM ya inicia automaticamente
  void update();               // Ejecuta el ciclo activo de la FSM en loop()
  bool isRunning() const;
  bool isGoalColorDetected();  // true solo con Rojo y checklist completo
  bool allRequiredColorsFound() const;
  void renderOled();           // OLED (canal 5): YAW|Color, F/L/R, paredes, ODO
  Orientation getHeading() const;
  void snapYawToNearest90();   // Corrige la deriva acumulada entre movimientos
  void alignWithFrontWall();   // Centrado longitudinal opcional contra pared frontal
  void triggerStuckRecovery(); // Micro-reversa segura y regreso a SCAN
  void restWithTelemetry(uint32_t durationMs);
};

#endif
