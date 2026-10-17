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

// Perfiles RGB normalizados obtenidos de la calibracion de la pista.
struct ColorRefRgb {
  TrackColor color;
  const char* name;
  float normR, normG, normB;
  float threshold;
};

const ColorRefRgb COLOR_TABLE_RGB[] = {
  {COLOR_CYAN,   "CIAN",       0.312f, 0.375f, 0.312f, 0.033f},
  {COLOR_YELLOW, "AMARILLO",   0.458f, 0.333f, 0.208f, 0.007f},
  {COLOR_ORANGE, "NARANJA",    0.579f, 0.263f, 0.158f, 0.044f},
  {COLOR_PINK,   "ROSA",       0.636f, 0.182f, 0.182f, 0.014f},
  {COLOR_RED,    "ROJO",       0.640f, 0.200f, 0.160f, 0.014f},
  {COLOR_GREEN,  "VERDE",      0.364f, 0.364f, 0.273f, 0.033f},
  {COLOR_FLOOR,  "PISO",       0.447f, 0.340f, 0.213f, 0.007f},
  {COLOR_LIME,   "VERDE LIMA", 0.421f, 0.368f, 0.211f, 0.019f}
};
constexpr uint8_t COLOR_TABLE_RGB_SIZE = sizeof(COLOR_TABLE_RGB) / sizeof(COLOR_TABLE_RGB[0]);

enum State { SCAN, DECIDE, TURN, ADVANCE, STOP };
enum Move { FORWARD, TURN_RIGHT, TURN_LEFT, TURN_180, ACTION_STOP };
enum Orientation { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

class Navigator {
 private:
  enum class TurnResult : uint8_t { IN_PROGRESS, COMPLETED, FAILED };

  State currentState;
  Move plannedMove;
  Orientation currentHeading;

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
  TrackColor readDetectedColor();  // Canal 1: vecino mas cercano 4D sobre COLOR_TABLE
  void calibrateGyroBias();
  void turn90(bool turnLeft, float targetDegrees = 90.0f);
  void startTurn180();
  TurnResult updateTurn();
  void beginTurnBrake();
  void refreshStationaryTelemetry();

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
