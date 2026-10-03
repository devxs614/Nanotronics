#include "Diagnostics.h"

void Diagnostics::begin() {
  enabled_ = DEBUG_ENABLED;
}

void Diagnostics::runTest(const char* name) {
  if (!enabled_) return;
  Serial.print(F("Running test: "));
  Serial.println(name);
}

void Diagnostics::printMenu() {
  Serial.println(F("D - MOTOR TEST"));
  Serial.println(F("D - ENCODER TEST"));
  Serial.println(F("D - IMU TEST"));
  Serial.println(F("D - VL53 TEST"));
  Serial.println(F("D - COLOR TEST"));
  Serial.println(F("D - QTR TEST"));
  Serial.println(F("D - SERVO TEST"));
  Serial.println(F("D - OLED TEST"));
  Serial.println(F("D - MECANUM TEST"));
}
