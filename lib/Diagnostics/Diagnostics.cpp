#include "Diagnostics.h"

void Diagnostics::begin() {
  enabled_ = DEBUG_ENABLED && !COMPETITION_MODE;
}

void Diagnostics::runTest(const char* name) {
  if (!enabled_) return;
  Serial.print(F("Running test: "));
  Serial.println(name);
}

void Diagnostics::printMenu() {
  if (!enabled_) return;
  Serial.println(F("D - MOTOR TEST"));
  Serial.println(F("E - ENCODER TEST"));
  Serial.println(F("I - IMU TEST"));
  Serial.println(F("V - VL53 TEST"));
  Serial.println(F("C - COLOR TEST"));
  Serial.println(F("Q - QTR TEST"));
  Serial.println(F("S - SERVO TEST"));
  Serial.println(F("O - OLED TEST"));
  Serial.println(F("M - MECANUM TEST"));
  Serial.println(F("Calibration: m/e/i/c/q/s; press x to stop QTR sampling."));
}
