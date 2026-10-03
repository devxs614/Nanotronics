#include "BallHandler.h"

void BallHandler::begin() {
  ballState_ = robot::BallState::UNKNOWN;
}

void BallHandler::update() {
  // Ball acquisition strategy: approach -> align -> grab -> verify -> secure.
}

bool BallHandler::hasBall() const {
  return ballState_ == robot::BallState::CAPTURED || ballState_ == robot::BallState::SECURED;
}

void BallHandler::setBallState(robot::BallState state) {
  ballState_ = state;
}
