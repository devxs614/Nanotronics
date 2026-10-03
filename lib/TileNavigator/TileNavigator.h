#ifndef TILE_NAVIGATOR_H
#define TILE_NAVIGATOR_H

#include <Arduino.h>
#include "config.h"

class TileNavigator {
public:
  void begin();
  void update();
  void reset();

private:
  bool active_; 
};

#endif
