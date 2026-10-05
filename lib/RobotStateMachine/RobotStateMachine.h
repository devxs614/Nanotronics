#ifndef ROBOT_STATE_MACHINE_H
#define ROBOT_STATE_MACHINE_H

#include <Arduino.h>
#include "config.h"
#include "competition_config.h"
#include "MazeNavigator.h"
#include "BallHandler.h"
#include "LineAvoidanceController.h"
#include "TileNavigator.h"
#include "CheckpointManager.h"
#include "RecoveryManager.h"
#include "Odometry.h"
#include "MecanumDrive.h"
#include "DisplayManager.h"

class RobotStateMachine {
public:
  void begin();
  void configure(robot::ActiveTrack track, MazeNavigator* maze, BallHandler* ball,
                 LineAvoidanceController* lineAvoidance, TileNavigator* tiles,
                 CheckpointManager* checkpoints, RecoveryManager* recovery,
                 Odometry* odometry, MecanumDrive* drive, DisplayManager* display);
  void update();
  robot::RobotMode getCurrentMode() const;
  robot::PistaBState getPistaBState() const;
  void setMode(robot::RobotMode mode);
  void reportLackOfProgress();

private:
  void updatePistaB();
  void updateProgressMonitor();
  void startSelectedTrack();
  robot::RobotMode currentMode_;
  robot::PistaBState pistaBState_;
  robot::ActiveTrack activeTrack_;
  MazeNavigator* maze_;
  BallHandler* ball_;
  LineAvoidanceController* lineAvoidance_;
  TileNavigator* tiles_;
  CheckpointManager* checkpoints_;
  RecoveryManager* recovery_;
  Odometry* odometry_;
  MecanumDrive* drive_;
  DisplayManager* display_;
  uint32_t stateStartedMs_;
  float sectionStartX_;
  float sectionStartY_;
  float progressReferenceX_;
  float progressReferenceY_;
  uint32_t lastProgressMs_;
  uint8_t progressSection_;
  bool progressMonitorInitialized_;
};

#endif
