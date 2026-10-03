#include "CheckpointManager.h"

void CheckpointManager::begin() {
  currentCheckpoint_ = 0;
}

void CheckpointManager::registerCheckpoint(uint8_t checkpoint) {
  currentCheckpoint_ = checkpoint;
}

uint8_t CheckpointManager::currentCheckpoint() const {
  return currentCheckpoint_;
}

void CheckpointManager::reset() {
  currentCheckpoint_ = 0;
}
