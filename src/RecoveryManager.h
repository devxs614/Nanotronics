#ifndef RECOVERY_MANAGER_H
#define RECOVERY_MANAGER_H

#include <Arduino.h>
#include "config.h"

class RecoveryManager {
public:
  void begin();
  void resetToCheckpoint();
  void setCheckpoint(uint8_t checkpoint);

private:
  uint8_t checkpoint_;
};

#endif
