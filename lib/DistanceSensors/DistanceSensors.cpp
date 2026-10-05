#include "DistanceSensors.h"

void DistanceSensors::begin() {
  frontReady_ = leftReady_ = rightReady_ = false;
  frontMm_ = leftMm_ = rightMm_ = 0;
  // 1. Apagar el sensor de color para liberar la dirección 0x29
  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, LOW);

  // 2. Colocar los 3 VL53L0X en estado de reset
  pinMode(DISTANCE_FRONT_XSHUT, OUTPUT);
  pinMode(DISTANCE_LEFT_XSHUT, OUTPUT);
  pinMode(DISTANCE_RIGHT_XSHUT, OUTPUT);

  digitalWrite(DISTANCE_FRONT_XSHUT, LOW);
  digitalWrite(DISTANCE_LEFT_XSHUT, LOW);
  digitalWrite(DISTANCE_RIGHT_XSHUT, LOW);
  delay(VL53_RESET_SETTLE_MS);

  // 3. Inicializar e integrar cada VL53L0X secuencialmente
  digitalWrite(DISTANCE_FRONT_XSHUT, HIGH);
  delay(VL53_START_SETTLE_MS);
  frontReady_ = frontSensor_.begin(VL53_FRONT_ADDRESS, false);
  if (frontReady_) frontReady_ = frontSensor_.startRangeContinuous(VL53_CONTINUOUS_PERIOD_MS);

  digitalWrite(DISTANCE_RIGHT_XSHUT, HIGH);
  delay(VL53_START_SETTLE_MS);
  rightReady_ = rightSensor_.begin(VL53_RIGHT_ADDRESS, false);
  if (rightReady_) rightReady_ = rightSensor_.startRangeContinuous(VL53_CONTINUOUS_PERIOD_MS);

  digitalWrite(DISTANCE_LEFT_XSHUT, HIGH);
  delay(VL53_START_SETTLE_MS);
  leftReady_ = leftSensor_.begin(VL53_LEFT_ADDRESS, false);
  if (leftReady_) leftReady_ = leftSensor_.startRangeContinuous(VL53_CONTINUOUS_PERIOD_MS);

  frontMm_ = 0;
  leftMm_ = 0;
  rightMm_ = 0;
}

void DistanceSensors::update() {
  if (!frontReady_ && !leftReady_ && !rightReady_) return;

  if (frontReady_) {
    if (frontSensor_.isRangeComplete()) {
      const uint16_t range = frontSensor_.readRangeResult();
      frontMm_ = frontSensor_.readRangeStatus() == 0 ? range : 0;
    }
  }

  if (rightReady_) {
    if (rightSensor_.isRangeComplete()) {
      const uint16_t range = rightSensor_.readRangeResult();
      rightMm_ = rightSensor_.readRangeStatus() == 0 ? range : 0;
    }
  }

  if (leftReady_) {
    if (leftSensor_.isRangeComplete()) {
      const uint16_t range = leftSensor_.readRangeResult();
      leftMm_ = leftSensor_.readRangeStatus() == 0 ? range : 0;
    }
  }
}

uint16_t DistanceSensors::readFront() const {
  return frontMm_;
}

uint16_t DistanceSensors::readLeft() const {
  return leftMm_;
}

uint16_t DistanceSensors::readRight() const {
  return rightMm_;
}

bool DistanceSensors::frontHealthy() const { return frontReady_ && frontMm_ != 0; }
bool DistanceSensors::leftHealthy() const { return leftReady_ && leftMm_ != 0; }
bool DistanceSensors::rightHealthy() const { return rightReady_ && rightMm_ != 0; }
