#ifndef CHECKPOINT_MANAGER_H
#define CHECKPOINT_MANAGER_H

#include <Arduino.h>
#include "config.h"

class Odometry;
class IMUManager;

class CheckpointManager {
public:
  void begin();
  void configure(Odometry* odometry, IMUManager* imu);
  void registerCheckpoint(uint8_t checkpoint);
  void registerCheckpoint(uint8_t checkpoint, uint8_t section, robot::ActiveTrack track);
  uint8_t currentCheckpoint() const;
  uint8_t currentSection() const;
  robot::ActiveTrack activeTrack() const;
  robot::RobotPose checkpointPose() const;
  void recordPathDirection(uint8_t direction);
  bool popReturnDirection(uint8_t& direction);
  void clearPath();
  void invalidatePath();
  bool pathReproducible() const;
  uint8_t pathLength() const;
  void reset();

private:
  Odometry* odometry_;
  IMUManager* imu_;
  uint8_t currentCheckpoint_;
  uint8_t currentSection_;
  robot::ActiveTrack activeTrack_;
  robot::RobotPose checkpointPose_;
  uint8_t path_[64];
  uint8_t pathLength_;
  bool pathReproducible_;
  void loadCheckpoint();
  void saveCheckpoint();
};

#endif
