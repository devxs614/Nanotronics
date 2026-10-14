#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Arduino.h>
#include <EEPROM.h>
#include <Adafruit_TCS34725.h>
#include "config.h"

namespace robot {

struct ColorProfile {
  float rNorm, gNorm, bNorm;
  uint16_t clearRaw;
};

struct ColorCalibrationData {
  ColorProfile cian;
  ColorProfile amarillo;
  ColorProfile naranja;
  ColorProfile rosa;
  ColorProfile rojo;
  ColorProfile verde;
  ColorProfile piso;
  bool isCalibrated;
};

}  // namespace robot

class ColorSensor {
public:
  static constexpr uint8_t PROFILE_COUNT = 7;

  void begin();
  void update();
  void readRaw();
  void readRGB();
  float calculateNormalizedRGB();
  robot::ColorClass calculateColorClassification();
  bool isValidColorSample() const;

  // Nombre corto (<= 5 caracteres) del color confirmado: Cian, Amar, Naran, Rosa, Rojo, Verde, Piso, Desc
  const char* getColorName() const;
  bool isCalibrated() const { return calibrationData_.isCalibrated; }

  robot::ColorReading latestReading;

private:
  enum Slot : int8_t {
    SLOT_NONE = -1,
    SLOT_CYAN = 0, SLOT_YELLOW, SLOT_ORANGE, SLOT_PINK, SLOT_RED, SLOT_GREEN, SLOT_FLOOR,
    SLOT_UNKNOWN = PROFILE_COUNT
  };

  void loadCalibration();
  const robot::ColorProfile& profile(uint8_t slot) const;
  int8_t classifyCalibrated();
  static int8_t slotFromClass(robot::ColorClass c);
  static robot::ColorClass classFromSlot(int8_t slot);

  Adafruit_TCS34725 tcs_;
  bool initialized_;
  robot::ColorClass candidateColor_;
  uint8_t consistentSamples_;
  robot::ColorCalibrationData calibrationData_;
  int8_t currentSlot_;
  int8_t candidateSlot_;
};

#endif
