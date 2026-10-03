#ifndef VISION_INTERFACE_H
#define VISION_INTERFACE_H

#include <Arduino.h>
#include "config.h"

class VisionInterface {
public:
  void begin();
  bool isAvailable() const;
  void update();

private:
  bool enabled_;
};

#endif
