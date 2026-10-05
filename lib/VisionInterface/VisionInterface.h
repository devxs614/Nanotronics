#ifndef VISION_INTERFACE_H
#define VISION_INTERFACE_H

#include <Arduino.h>
#include "config.h"

class VisionInterface {
public:
  void begin();
  bool isAvailable() const;
  void update();
  bool hasMarker() const;
  bool consumeNewMarker(uint16_t& markerId);
  uint16_t markerId() const;

private:
  bool enabled_;
  char input_[16];
  uint8_t inputLength_;
  uint16_t markerId_;
  bool hasMarker_;
  bool newMarker_;
  uint32_t lastMarkerMs_;
  void parseLine();
};

#endif
