#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include <EEPROM.h>

// Formato que guarda tests_2.0/09_test_color_calibration_eeprom.cpp en la EEPROM (direccion 0):
// codigo magic + una firma por color (Cian, Amarillo, Naranja, Rosa, Rojo, Verde, Piso).
struct ColorSignature {
  uint16_t r, g, b, c;   // Promedio crudo
  float rn, gn, bn;      // RGB normalizado
  float threshold;       // Distancia maxima aceptada (espacio normalizado)
  uint8_t valid;
};

constexpr uint8_t COLOR_SLOT_COUNT = 7;
constexpr uint8_t COLOR_SLOT_RED = 4;
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

// Tabla 4D: RGB normalizado (r/(r+g+b), ...) + intensidad normC = C/50
struct ColorRef4D {
  const char* name;
  float normR, normG, normB, normC;
  bool isGoal;
};

const ColorRef4D COLOR_TABLE_4D[] = {
  {"Cian",       0.312f, 0.375f, 0.312f, 0.240f, false},  // C:12
  {"Amarillo",   0.458f, 0.333f, 0.208f, 0.400f, false},  // C:20
  {"Naranja",    0.579f, 0.263f, 0.158f, 0.300f, false},  // C:15
  {"Rosa",       0.636f, 0.182f, 0.182f, 0.620f, false},  // C:31
  {"Rojo",       0.640f, 0.200f, 0.160f, 0.400f, true },  // C:20 (META)
  {"Verde",      0.364f, 0.364f, 0.273f, 0.160f, false},  // C:8
  {"Piso",       0.447f, 0.340f, 0.213f, 0.880f, false},  // C:44
  {"Verde Lima", 0.421f, 0.368f, 0.211f, 0.320f, false}   // C:16
};
constexpr uint8_t COLOR_TABLE_4D_SIZE = sizeof(COLOR_TABLE_4D) / sizeof(COLOR_TABLE_4D[0]);
constexpr float COLOR_MAX_DISTANCE_4D = 0.25f;

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
  char currentColorName[12];  // Ultimo color clasificado (Rojo, Cian, ..., Desc)

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

  Move decideRightWallRule();
  String readDetectedColor(bool& outIsGoal);  // Canal 1: vecino mas cercano sobre COLOR_TABLE
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
  bool isGoalColorDetected();  // true si el TCS34725 (canal 1) clasifica como Rojo
  void renderOled();           // OLED (canal 5): YAW|Color, F/L/R, paredes, ODO
  Orientation getHeading() const;
  void restWithTelemetry(uint32_t durationMs);
};

#endif
