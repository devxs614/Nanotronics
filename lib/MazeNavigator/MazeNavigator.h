#ifndef MAZE_NAVIGATOR_H
#define MAZE_NAVIGATOR_H

#include <Arduino.h>
#include "config.h"
#include "DistanceSensors.h"
#include "ColorSensor.h"
#include "IMUManager.h"
#include "MecanumDrive.h"
#include "DisplayManager.h"
#include "CheckpointManager.h"
#include "Odometry.h"
#include "PIDController.h"

class MazeNavigator {
public:
  void begin();
  void configure(DistanceSensors* distance, ColorSensor* color, IMUManager* imu,
                 MecanumDrive* drive, DisplayManager* display, CheckpointManager* checkpoints,
                 Odometry* odometry);
  void update();
  void reset();
  robot::PistaAState state() const;

private:
  void transition(robot::MazeState next);
  void beginTurn(float deltaDeg);
  void updateReturn();
  DistanceSensors* distance_;
  ColorSensor* color_;
  IMUManager* imu_;
  MecanumDrive* drive_;
  DisplayManager* display_;
  CheckpointManager* checkpoints_;
  Odometry* odometry_;
  PIDController headingPid_;
  robot::PistaAState currentState_;
  robot::MazeState mazeState_;
  float targetYaw_;
  float moveStartX_;
  float moveStartY_;
  float returnTargetYaw_;
  uint8_t returnPhase_;
  uint8_t returnDirection_;
  bool returning_;
  uint32_t stateStartedMs_;
  uint32_t colorShownAtMs_;
};

#endif
