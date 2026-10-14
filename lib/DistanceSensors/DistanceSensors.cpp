#include "DistanceSensors.h"

void DistanceSensors::begin() {
  frontReady_ = leftReady_ = rightReady_ = false;
  frontMm_ = leftMm_ = rightMm_ = 0;
  // 1. Apagar el sensor de color para liberar la dirección 0x29
  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, LOW);

  // 2. Inicializar cada VL53L0X a traves del MUX (sin XSHUT)
  selectI2CChannel(MUX_CH_FRONT);
  delay(VL53_START_SETTLE_MS);
  frontReady_ = frontSensor_.begin(0x29, false);
  if (frontReady_) frontReady_ = frontSensor_.startRangeContinuous(VL53_CONTINUOUS_PERIOD_MS);

  selectI2CChannel(MUX_CH_RIGHT);
  delay(VL53_START_SETTLE_MS);
  rightReady_ = rightSensor_.begin(0x29, false);
  if (rightReady_) rightReady_ = rightSensor_.startRangeContinuous(VL53_CONTINUOUS_PERIOD_MS);

  selectI2CChannel(MUX_CH_LEFT);
  delay(VL53_START_SETTLE_MS);
  leftReady_ = leftSensor_.begin(0x29, false);
  if (leftReady_) leftReady_ = leftSensor_.startRangeContinuous(VL53_CONTINUOUS_PERIOD_MS);
  frontMm_ = 0;
  leftMm_ = 0;
  rightMm_ = 0;
}

void DistanceSensors::update() {
  if (!frontReady_ && !leftReady_ && !rightReady_) return;

  if (frontReady_) {
    selectI2CChannel(MUX_CH_FRONT);
    if (frontSensor_.isRangeComplete()) {
      const uint16_t range = frontSensor_.readRange();
      frontMm_ = frontSensor_.readRangeStatus() == 0 ? range : 0;
    }
  }

  if (rightReady_) {
    selectI2CChannel(MUX_CH_RIGHT);
    if (rightSensor_.isRangeComplete()) {
      const uint16_t range = rightSensor_.readRange();
      rightMm_ = rightSensor_.readRangeStatus() == 0 ? range : 0;
    }
  }

  if (leftReady_) {
    selectI2CChannel(MUX_CH_LEFT);
    if (leftSensor_.isRangeComplete()) {
      const uint16_t range = leftSensor_.readRange();
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
