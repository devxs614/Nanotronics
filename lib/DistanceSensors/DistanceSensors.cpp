#include "DistanceSensors.h"

void DistanceSensors::begin() {
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
  delay(50);

  // 3. Inicializar e integrar cada VL53L0X secuencialmente
  digitalWrite(DISTANCE_FRONT_XSHUT, HIGH);
  delay(20);
  frontSensor_.begin(VL53_FRONT_ADDRESS, false);
  frontSensor_.startRangeContinuous();

  digitalWrite(DISTANCE_RIGHT_XSHUT, HIGH);
  delay(20);
  rightSensor_.begin(VL53_RIGHT_ADDRESS, false);
  rightSensor_.startRangeContinuous();

  digitalWrite(DISTANCE_LEFT_XSHUT, HIGH);
  delay(20);
  leftSensor_.begin(VL53_LEFT_ADDRESS, false);
  leftSensor_.startRangeContinuous();

  frontMm_ = 0;
  leftMm_ = 0;
  rightMm_ = 0;
  initialized_ = true;
}

void DistanceSensors::update() {
  if (!initialized_) return;
  VL53L0X_RangingMeasurementData_t measure;

  frontSensor_.getSingleRangingMeasurement(&measure, false);
  frontMm_ = measure.RangeMilliMeter;

  rightSensor_.getSingleRangingMeasurement(&measure, false);
  rightMm_ = measure.RangeMilliMeter;

  leftSensor_.getSingleRangingMeasurement(&measure, false);
  leftMm_ = measure.RangeMilliMeter;
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
