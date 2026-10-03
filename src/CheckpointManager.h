#ifndef CHECKPOINT_MANAGER_H
#define CHECKPOINT_MANAGER_H

#include <Arduino.h>
#include "config.h"

class CheckpointManager {
public:
  void begin();
  void registerCheckpoint(uint8_t checkpoint);
  uint8_t currentCheckpoint() const;
  void reset();

private:
  uint8_t currentCheckpoint_;
};

#endif
