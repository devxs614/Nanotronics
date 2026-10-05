#ifndef BALL_HANDLER_H
#define BALL_HANDLER_H

#include <Arduino.h>
#include "config.h"
#include "DistanceSensors.h"
#include "GripperController.h"
#include "MecanumDrive.h"
#include "Odometry.h"

class BallHandler {
public:
  void begin();
  void update();
  bool hasBall() const;
  void setBallState(robot::BallState state);
  void configure(DistanceSensors* distance, GripperController* gripper, MecanumDrive* drive,
                 Odometry* odometry);
  robot::BallHandlerState state() const;
  robot::BallState ballState() const;
  void release();

private:
  void transition(robot::BallHandlerState next);
  DistanceSensors* distance_;
  GripperController* gripper_;
  MecanumDrive* drive_;
  Odometry* odometry_;
  robot::BallHandlerState state_;
  robot::BallState ballState_;
  uint32_t stateStartedMs_;
  uint32_t alignPhaseStartedMs_;
  uint16_t captureDistanceMm_;
  uint16_t alignBaselineMm_;
  float secureStartX_;
  float secureStartY_;
  uint8_t alignPhase_;
  bool objectValidated_;
  bool secureMotionStarted_;
};

#endif
