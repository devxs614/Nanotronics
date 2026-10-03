#include "Odometry.h"

void Odometry::begin() {
  x_ = 0.0f;
  y_ = 0.0f;
  theta_ = 0.0f;
}

void Odometry::update() {
  // Integrate wheel velocity and heading to update the local execution pose.
}

float Odometry::x() const {
  return x_;
}

float Odometry::y() const {
  return y_;
}

float Odometry::theta() const {
  return theta_;
}
