#include "RecoveryManager.h"
#include "CheckpointManager.h"
#include "Odometry.h"
#include <EEPROM.h>

void RecoveryManager::begin() {
  checkpoint_ = 0;
  lopSection_ = 0;
  lopCount_ = 0;
  forcedTransition_ = false;
  checkpoints_ = nullptr;
  odometry_ = nullptr;
  loadLopCounter();
}

void RecoveryManager::configure(CheckpointManager* checkpoints, Odometry* odometry) {
  checkpoints_ = checkpoints;
  odometry_ = odometry;
}

void RecoveryManager::resetToCheckpoint() {
  if (odometry_ == nullptr) return;
  if (forcedTransition_) {
    odometry_->resetPose(0.0f, 0.0f, odometry_->theta());
    forcedTransition_ = false;
    return;
  }
  if (checkpoints_ == nullptr) return;
  const robot::RobotPose pose = checkpoints_->checkpointPose();
  odometry_->resetPose(pose.xMm, pose.yMm, pose.headingDeg);
  checkpoint_ = checkpoints_->currentCheckpoint();
}

void RecoveryManager::setCheckpoint(uint8_t checkpoint) {
  checkpoint_ = checkpoint;
}

bool RecoveryManager::reportLackOfProgress(uint8_t section) {
  if (section != lopSection_) {
    lopSection_ = section;
    lopCount_ = 0;
  }
  if (lopCount_ < 255) ++lopCount_;
  saveLopCounter();
  return lopCount_ >= LOP_LIMIT_PER_SECTION;
}

void RecoveryManager::requestForcedTransition() {
  forcedTransition_ = true;
}

uint8_t RecoveryManager::lopCount(uint8_t section) const {
  return section == lopSection_ ? lopCount_ : 0;
}

void RecoveryManager::resetLopCounters() {
  lopSection_ = 0;
  lopCount_ = 0;
  saveLopCounter();
}

void RecoveryManager::loadLopCounter() {
  const uint16_t base = RECOVERY_EEPROM_BASE;
  const uint8_t magic = EEPROM.read(base);
  const uint8_t section = EEPROM.read(base + 1);
  const uint8_t count = EEPROM.read(base + 2);
  const uint8_t checksum = EEPROM.read(base + 3);
  if (magic == 0xC7 && checksum == static_cast<uint8_t>(magic ^ section ^ count ^ 0x5A)) {
    lopSection_ = section;
    lopCount_ = count;
  }
}

void RecoveryManager::saveLopCounter() {
  const uint16_t base = RECOVERY_EEPROM_BASE;
  const uint8_t magic = 0xC7;
  EEPROM.update(base, 0);
  EEPROM.update(base + 1, lopSection_);
  EEPROM.update(base + 2, lopCount_);
  EEPROM.update(base + 3, static_cast<uint8_t>(magic ^ lopSection_ ^ lopCount_ ^ 0x5A));
  EEPROM.update(base, magic);
}
