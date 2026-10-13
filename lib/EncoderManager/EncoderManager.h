#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include <Arduino.h>
#include "config.h"

extern volatile int32_t encoderTicks[4];

class EncoderManager {
public:
  void begin();
  void update();
  int32_t getTicks(uint8_t motorIdx) const;
  void resetTicks();
  float getRPM(uint8_t motor) const;
  float getDistanceMm(uint8_t motor) const;
  float getVelocityMmPerSec(uint8_t motor) const;
  void reset();

private:
  int32_t sampleTicks_[4];
  float rpm_[4];
  uint32_t lastSampleMs_;
};

#endif
