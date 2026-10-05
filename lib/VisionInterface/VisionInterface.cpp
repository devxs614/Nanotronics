#include "VisionInterface.h"

void VisionInterface::begin() {
  enabled_ = ENABLE_ARUCO;
  inputLength_ = 0;
  markerId_ = 0;
  hasMarker_ = false;
  newMarker_ = false;
  lastMarkerMs_ = 0;
  if (enabled_) Serial2.begin(115200);
}

bool VisionInterface::isAvailable() const {
  return enabled_;
}

void VisionInterface::update() {
  if (!enabled_) return;
  while (Serial2.available() > 0) {
    const char value = static_cast<char>(Serial2.read());
    if (value == '\n' || value == '\r') {
      if (inputLength_ > 0) parseLine();
      inputLength_ = 0;
    } else if (inputLength_ < sizeof(input_) - 1) {
      input_[inputLength_++] = value;
      input_[inputLength_] = '\0';
    } else {
      inputLength_ = 0;
    }
  }
}

bool VisionInterface::hasMarker() const {
  return hasMarker_ && millis() - lastMarkerMs_ <= ARUCO_MARKER_TIMEOUT_MS;
}

bool VisionInterface::consumeNewMarker(uint16_t& markerId) {
  if (!newMarker_) return false;
  newMarker_ = false;
  markerId = markerId_;
  return true;
}

uint16_t VisionInterface::markerId() const { return markerId_; }

void VisionInterface::parseLine() {
  if (strncmp(input_, "ARUCO:", 6) != 0 || input_[6] == '\0') return;
  char* end = nullptr;
  const long parsed = strtol(input_ + 6, &end, 10);
  if (end == input_ + 6 || *end != '\0' || parsed < 0 || parsed > 65535) return;
  markerId_ = static_cast<uint16_t>(parsed);
  hasMarker_ = true;
  newMarker_ = true;
  lastMarkerMs_ = millis();
}
