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

enum State { SCAN, DECIDE, TURN, ADVANCE, STOP };
enum Move { FORWARD, TURN_RIGHT, TURN_LEFT, TURN_180, ACTION_STOP };
enum Orientation { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

class Navigator {
 private:
  State currentState;
  Move plannedMove;
  Orientation currentHeading;

  // Banderas de lectura de pared
  bool freeF, freeL, freeR;

  ColorProfile redProfile;  // Color de meta cargado de la EEPROM

  Move decideRightWallRule();

 public:
  Navigator();
  void begin();
  void start();                // Inicia la navegacion (comando 'g')
  void update();               // Ejecuta el ciclo activo de la FSM en loop()
  void stop();
  bool isRunning() const;
  bool isGoalColorDetected();  // Compara el TCS34725 (canal 1) contra redProfile
  Orientation getHeading() const;
};

#endif
