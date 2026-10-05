#include "CheckpointManager.h"
#include "Odometry.h"
#include "IMUManager.h"
#include <EEPROM.h>

void CheckpointManager::begin() {
  currentCheckpoint_ = 0;
  currentSection_ = 0;
  activeTrack_ = robot::ActiveTrack::PISTA_B;
  checkpointPose_ = {0.0f, 0.0f, 0.0f};
  pathLength_ = 0;
  pathReproducible_ = true;
  odometry_ = nullptr;
  imu_ = nullptr;
  loadCheckpoint();
}

void CheckpointManager::configure(Odometry* odometry, IMUManager* imu) {
  odometry_ = odometry;
  imu_ = imu;
}

void CheckpointManager::registerCheckpoint(uint8_t checkpoint) {
  currentCheckpoint_ = checkpoint;
  currentSection_ = checkpoint;
  if (odometry_ != nullptr) checkpointPose_ = odometry_->pose();
  if (imu_ != nullptr && imu_->healthy()) checkpointPose_.headingDeg = imu_->getYaw();
  saveCheckpoint();
}

void CheckpointManager::registerCheckpoint(uint8_t checkpoint, uint8_t section, robot::ActiveTrack track) {
  activeTrack_ = track;
  currentSection_ = section;
  registerCheckpoint(checkpoint);
}

uint8_t CheckpointManager::currentCheckpoint() const {
  return currentCheckpoint_;
}

uint8_t CheckpointManager::currentSection() const { return currentSection_; }
robot::ActiveTrack CheckpointManager::activeTrack() const { return activeTrack_; }
robot::RobotPose CheckpointManager::checkpointPose() const { return checkpointPose_; }

void CheckpointManager::recordPathDirection(uint8_t direction) {
  if (pathLength_ < sizeof(path_)) path_[pathLength_++] = direction;
  else pathReproducible_ = false;
}

bool CheckpointManager::popReturnDirection(uint8_t& direction) {
  if (pathLength_ == 0) return false;
  direction = path_[--pathLength_];
  return true;
}

void CheckpointManager::clearPath() {
  pathLength_ = 0;
  pathReproducible_ = true;
}

void CheckpointManager::invalidatePath() { pathReproducible_ = false; }
bool CheckpointManager::pathReproducible() const { return pathReproducible_; }
uint8_t CheckpointManager::pathLength() const { return pathLength_; }

void CheckpointManager::reset() {
  currentCheckpoint_ = 0;
  currentSection_ = 0;
  checkpointPose_ = {0.0f, 0.0f, 0.0f};
  clearPath();
  EEPROM.update(CHECKPOINT_EEPROM_BASE, 0);
}

void CheckpointManager::loadCheckpoint() {
  const uint16_t base = CHECKPOINT_EEPROM_BASE;
  if (EEPROM.read(base) != 0xC8) return;
  uint8_t checksum = 0;
  for (uint8_t offset = 1; offset <= 15; ++offset) checksum ^= EEPROM.read(base + offset);
  if (checksum != EEPROM.read(base + 16)) return;

  currentCheckpoint_ = EEPROM.read(base + 1);
  currentSection_ = EEPROM.read(base + 2);
  activeTrack_ = EEPROM.read(base + 3) == 0
    ? robot::ActiveTrack::PISTA_A : robot::ActiveTrack::PISTA_B;
  EEPROM.get(base + 4, checkpointPose_.xMm);
  EEPROM.get(base + 8, checkpointPose_.yMm);
  EEPROM.get(base + 12, checkpointPose_.headingDeg);
}

void CheckpointManager::saveCheckpoint() {
  const uint16_t base = CHECKPOINT_EEPROM_BASE;
  EEPROM.update(base, 0);
  EEPROM.update(base + 1, currentCheckpoint_);
  EEPROM.update(base + 2, currentSection_);
  EEPROM.update(base + 3, activeTrack_ == robot::ActiveTrack::PISTA_A ? 0 : 1);
  EEPROM.put(base + 4, checkpointPose_.xMm);
  EEPROM.put(base + 8, checkpointPose_.yMm);
  EEPROM.put(base + 12, checkpointPose_.headingDeg);
  uint8_t checksum = 0;
  for (uint8_t offset = 1; offset <= 15; ++offset) checksum ^= EEPROM.read(base + offset);
  EEPROM.update(base + 16, checksum);
  EEPROM.update(base, 0xC8);
}
