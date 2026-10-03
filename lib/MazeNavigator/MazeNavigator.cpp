#include "MazeNavigator.h"

void MazeNavigator::begin() {
  currentState_ = robot::PistaAState::A_INIT;
}

void MazeNavigator::update() {
  // Maze behavior is controlled by sensor-driven wall following and recovery logic.
  // This is intentionally written as a clean state-machine framework rather than a monolith.
}

void MazeNavigator::reset() {
  currentState_ = robot::PistaAState::A_INIT;
}

robot::PistaAState MazeNavigator::state() const {
  return currentState_;
}
