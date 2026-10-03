#include "VisionInterface.h"

void VisionInterface::begin() {
  enabled_ = ENABLE_ARUCO;
}

bool VisionInterface::isAvailable() const {
  return enabled_;
}

void VisionInterface::update() {
  // ArUco and camera support remain optional features that require additional hardware.
}
