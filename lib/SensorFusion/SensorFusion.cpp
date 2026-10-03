#include "SensorFusion.h"

void SensorFusion::begin() {
  headingDeg_ = 0.0f;
}

void SensorFusion::update() {
  // Sensor fusion logic is intentionally lightweight and keeps the compute burden low.
}

float SensorFusion::headingDeg() const {
  return headingDeg_;
}
