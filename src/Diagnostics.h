#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <Arduino.h>
#include "config.h"

class Diagnostics {
public:
  void begin();
  void runTest(const char* name);
  void printMenu();

private:
  bool enabled_;
};

#endif
