#include "TileNavigator.h"

void TileNavigator::begin() {
  active_ = false;
}

void TileNavigator::update() {
  // Follows the track-relative motion conversion principle: color -> track direction -> robot frame -> execute one cell.
}

void TileNavigator::reset() {
  active_ = false;
}
