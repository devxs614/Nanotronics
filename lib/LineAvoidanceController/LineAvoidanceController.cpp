#include "LineAvoidanceController.h"

void LineAvoidanceController::begin() {
  whiteLineDetected_ = false;
  corridorClear_ = true;
}

void LineAvoidanceController::configure(LineSensor* sensor, MecanumDrive* drive) {
  sensor_ = sensor;
  drive_ = drive;
}

void LineAvoidanceController::update() {
  if (sensor_ == nullptr || drive_ == nullptr) return;
  const robot::LineObservation& line = sensor_->observation();
  whiteLineDetected_ = line.detected;
  corridorClear_ = !line.detected;

  if (!line.detected) {
    drive_->driveForward(LINE_AVOID_SPEED_MM_S);
    return;
  }

  if (line.leftBlocked && !line.rightBlocked) {
    drive_->strafeRight(LINE_AVOID_SPEED_MM_S);
  } else if (line.rightBlocked && !line.leftBlocked) {
    drive_->strafeLeft(LINE_AVOID_SPEED_MM_S);
  } else if (line.normalizedPosition >= 0.0f) {
    drive_->strafeLeft(LINE_AVOID_SPEED_MM_S);
  } else {
    drive_->strafeRight(LINE_AVOID_SPEED_MM_S);
  }
}

bool LineAvoidanceController::whiteLineDetected() const {
  return whiteLineDetected_;
}

bool LineAvoidanceController::corridorClear() const { return corridorClear_; }
