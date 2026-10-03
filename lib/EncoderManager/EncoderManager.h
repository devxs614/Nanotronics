#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include <Arduino.h>
#include "config.h"

extern volatile int32_t encoderTicks[4];

class EncoderManager {
public:
  void begin();
  void update();
  int32_t getTicks(uint8_t motor) const;
  float getRPM(uint8_t motor) const;
  float getDistanceMm(uint8_t motor) const;
  float getVelocityMmPerSec(uint8_t motor) const;
  void reset();

  static EncoderManager* instance_;
  static void processPortB(uint8_t portState);
  static void processPortJ(uint8_t portState);
  static void processPortH(uint8_t portState);
  static void processQuadrature(uint8_t motor, uint8_t state);

private:
  static volatile uint8_t previousPortB_;
  static volatile uint8_t previousPortJ_;
  static volatile uint8_t previousPortH_;

  int32_t previousTicks[4];
  uint32_t previousMicros[4];
};

#endif
