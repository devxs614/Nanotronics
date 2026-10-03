#include "RobotStateMachine.h"

void RobotStateMachine::begin() {
  currentMode_ = robot::RobotMode::STARTUP;
}

void RobotStateMachine::update() {
  // The high-level state transition logic is intentionally centralized here.
  // For the hardware integration phase, the state is advanced in a simple and safe manner.
  if (currentMode_ == robot::RobotMode::STARTUP) {
    currentMode_ = robot::RobotMode::WAIT_FOR_START;
  }
}

robot::RobotMode RobotStateMachine::getCurrentMode() const {
  return currentMode_;
}

void RobotStateMachine::setMode(robot::RobotMode mode) {
  currentMode_ = mode;
}
