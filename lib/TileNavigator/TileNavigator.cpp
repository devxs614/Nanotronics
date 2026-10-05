#include "TileNavigator.h"

void TileNavigator::begin() {
  active_ = false;
  finished_ = false;
  originYaw_ = 0.0f;
  targetX_ = 0.0f;
  targetY_ = 0.0f;
  lastArrivalMs_ = 0;
}

void TileNavigator::configure(ColorSensor* color, IMUManager* imu, Odometry* odometry,
                              MecanumDrive* drive, DisplayManager* display) {
  color_ = color;
  imu_ = imu;
  odometry_ = odometry;
  drive_ = drive;
  display_ = display;
  originYaw_ = imu_ != nullptr ? imu_->getYaw() : 0.0f;
}

void TileNavigator::update() {
  if (finished_ || color_ == nullptr || odometry_ == nullptr || drive_ == nullptr) return;

  if (!active_) {
    drive_->stop();
    if (millis() - lastArrivalMs_ < TILE_ARRIVAL_COOLDOWN_MS) return;

    const robot::ColorClass color = color_->latestReading.color;
    if (color == robot::ColorClass::COLOR_RED) {
      finished_ = true;
      if (display_ != nullptr) display_->setSensor("FINAL RED");
      return;
    }

    const robot::RobotPose pose = odometry_->pose();
    targetX_ = pose.xMm;
    targetY_ = pose.yMm;
    switch (color) {
      case robot::ColorClass::COLOR_CYAN: targetX_ += MOVE_ONE_CELL_MM; break;
      case robot::ColorClass::COLOR_YELLOW: targetX_ -= MOVE_ONE_CELL_MM; break;
      case robot::ColorClass::COLOR_ORANGE: targetY_ += MOVE_ONE_CELL_MM; break;
      case robot::ColorClass::COLOR_PINK: targetY_ -= MOVE_ONE_CELL_MM; break;
      default: return;
    }
    active_ = true;
    if (display_ != nullptr) display_->showDirection(color);
  }

  const robot::RobotPose pose = odometry_->pose();
  const float dx = targetX_ - pose.xMm;
  const float dy = targetY_ - pose.yMm;
  const float distance = sqrtf(dx * dx + dy * dy);
  if (distance <= ODOMETRY_CELL_TOLERANCE_MM) {
    drive_->stop();
    active_ = false;
    lastArrivalMs_ = millis();
    return;
  }

  const float heading = (imu_ != nullptr ? imu_->getYaw() - originYaw_ : pose.headingDeg) * 0.0174532925f;
  const float robotRight = dx * cosf(heading) + dy * sinf(heading);
  const float robotForward = -dx * sinf(heading) + dy * cosf(heading);
  const float vx = NAVIGATION_SPEED_MM_S * robotRight / distance;
  const float vy = NAVIGATION_SPEED_MM_S * robotForward / distance;
  drive_->drive(vx, vy, 0.0f);
}

void TileNavigator::reset() {
  active_ = false;
  finished_ = false;
  lastArrivalMs_ = 0;
}

bool TileNavigator::finished() const { return finished_; }
bool TileNavigator::moving() const { return active_; }
