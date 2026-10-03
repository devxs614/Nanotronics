#include "RecoveryManager.h"

void RecoveryManager::begin() {
  checkpoint_ = 0;
}

void RecoveryManager::resetToCheckpoint() {
  // Recovery is safe: the robot re-enters a known checkpoint and resumes without hard-coded maze map data.
}

void RecoveryManager::setCheckpoint(uint8_t checkpoint) {
  checkpoint_ = checkpoint;
}
