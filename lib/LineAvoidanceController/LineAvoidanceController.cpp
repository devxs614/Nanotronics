#include "LineAvoidanceController.h"

void LineAvoidanceController::begin() {
  whiteLineDetected_ = false;
}

void LineAvoidanceController::update() {
  // The actual detection comes from LineSensor and a local strategy that tries to find the clear corridor.
}

bool LineAvoidanceController::whiteLineDetected() const {
  return whiteLineDetected_;
}
