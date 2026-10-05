#ifndef RECOVERY_MANAGER_H
#define RECOVERY_MANAGER_H

#include <Arduino.h>
#include "config.h"

class CheckpointManager;
class Odometry;

class RecoveryManager {
public:
  void begin();
  void configure(CheckpointManager* checkpoints, Odometry* odometry);
  void resetToCheckpoint();
  void setCheckpoint(uint8_t checkpoint);
  bool reportLackOfProgress(uint8_t section);
  void requestForcedTransition();
  uint8_t lopCount(uint8_t section) const;
  void resetLopCounters();

private:
  CheckpointManager* checkpoints_;
  Odometry* odometry_;
  uint8_t checkpoint_;
  uint8_t lopSection_;
  uint8_t lopCount_;
  bool forcedTransition_;
  void loadLopCounter();
  void saveLopCounter();
};

#endif
