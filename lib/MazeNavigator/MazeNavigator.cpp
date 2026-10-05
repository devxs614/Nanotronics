#include "MazeNavigator.h"

void MazeNavigator::begin() {
  currentState_ = robot::PistaAState::A_INIT;
  mazeState_ = robot::MazeState::MAZE_INIT;
  targetYaw_ = 0.0f;
  moveStartX_ = moveStartY_ = returnTargetYaw_ = 0.0f;
  returnPhase_ = 0;
  returnDirection_ = 0;
  returning_ = false;
  stateStartedMs_ = millis();
  colorShownAtMs_ = 0;
}

void MazeNavigator::configure(DistanceSensors* distance, ColorSensor* color, IMUManager* imu,
                              MecanumDrive* drive, DisplayManager* display,
                              CheckpointManager* checkpoints, Odometry* odometry) {
  distance_ = distance;
  color_ = color;
  imu_ = imu;
  drive_ = drive;
  display_ = display;
  checkpoints_ = checkpoints;
  odometry_ = odometry;
  targetYaw_ = imu_ != nullptr ? imu_->getYaw() : 0.0f;
  headingPid_.setTunings(KP_HEADING, KI_HEADING, KD_HEADING);
  headingPid_.setSampleTimeMs(20);
  headingPid_.setOutputLimits(-0.8f, 0.8f);
}

void MazeNavigator::update() {
  if (distance_ == nullptr || color_ == nullptr || drive_ == nullptr) return;

  if (mazeState_ == robot::MazeState::MAZE_INIT) {
    currentState_ = robot::PistaAState::A_MAZE_NAVIGATION;
    transition(robot::MazeState::MAZE_FIND_PATH);
  }

  if (!returning_ && color_->latestReading.color == robot::ColorClass::COLOR_RED) {
    drive_->stop();
    if (mazeState_ == robot::MazeState::MAZE_FORWARD && odometry_ != nullptr &&
        checkpoints_ != nullptr) {
      const robot::RobotPose pose = odometry_->pose();
      const float dx = pose.xMm - moveStartX_;
      const float dy = pose.yMm - moveStartY_;
      if (sqrtf(dx * dx + dy * dy) >= MOVE_ONE_CELL_MM - ODOMETRY_CELL_TOLERANCE_MM) {
        const uint8_t cardinal = static_cast<uint8_t>(lroundf(targetYaw_ / 90.0f)) % 4;
        checkpoints_->recordPathDirection(cardinal);
      } else {
        checkpoints_->invalidatePath();
      }
    }
    if (ENABLE_RETURN_BONUS && checkpoints_ != nullptr &&
        checkpoints_->pathReproducible() && checkpoints_->pathLength() > 0) {
      returning_ = true;
      returnPhase_ = 0;
      currentState_ = robot::PistaAState::A_RETURN_BONUS;
      transition(robot::MazeState::MAZE_RETURN);
    } else {
      currentState_ = robot::PistaAState::A_FINISH;
      mazeState_ = robot::MazeState::MAZE_FINISH;
    }
    return;
  }

  if (color_->latestReading.color != robot::ColorClass::COLOR_NONE &&
      color_->latestReading.color != robot::ColorClass::COLOR_UNKNOWN &&
      display_ != nullptr && millis() - colorShownAtMs_ >= COLOR_DISPLAY_TIME_MS) {
    display_->showColor(color_->latestReading.color);
    colorShownAtMs_ = millis();
  }

  switch (mazeState_) {
    case robot::MazeState::MAZE_FIND_PATH:
      currentState_ = robot::PistaAState::A_MAZE_NAVIGATION;
      if (!distance_->frontHealthy()) {
        drive_->stop();
        transition(robot::MazeState::MAZE_RECOVERY);
      } else if (distance_->readFront() <= FRONT_STOP_MM) {
        drive_->stop();
        transition(robot::MazeState::MAZE_DECIDE_TURN);
      } else {
        const robot::RobotPose pose = odometry_ != nullptr ? odometry_->pose() : robot::RobotPose{0, 0, targetYaw_};
        moveStartX_ = pose.xMm;
        moveStartY_ = pose.yMm;
        targetYaw_ = imu_ != nullptr ? imu_->getYaw() : targetYaw_;
        transition(robot::MazeState::MAZE_FORWARD);
      }
      break;
    case robot::MazeState::MAZE_FORWARD: {
      const robot::RobotPose pose = odometry_ != nullptr ? odometry_->pose() : robot::RobotPose{0, 0, targetYaw_};
      const float dx = pose.xMm - moveStartX_;
      const float dy = pose.yMm - moveStartY_;
      if (sqrtf(dx * dx + dy * dy) >= MOVE_ONE_CELL_MM - ODOMETRY_CELL_TOLERANCE_MM) {
        drive_->stop();
        if (checkpoints_ != nullptr) {
          const float heading = targetYaw_ < 0.0f ? targetYaw_ + 360.0f : targetYaw_;
          const uint8_t cardinal = static_cast<uint8_t>(lroundf(heading / 90.0f)) % 4;
          checkpoints_->recordPathDirection(cardinal);
        }
        transition(robot::MazeState::MAZE_DECIDE_TURN);
      } else if (distance_->frontHealthy() && distance_->readFront() <= FRONT_STOP_MM) {
        drive_->stop();
        if (checkpoints_ != nullptr) checkpoints_->invalidatePath();
        transition(robot::MazeState::MAZE_DECIDE_TURN);
      } else if (imu_ == nullptr || !imu_->healthy()) {
        drive_->stop();
        transition(robot::MazeState::MAZE_RECOVERY);
      } else {
        float headingError = targetYaw_ - imu_->getYaw();
        if (headingError > 180.0f) headingError -= 360.0f;
        if (headingError < -180.0f) headingError += 360.0f;
        const float omega = headingPid_.update(-headingError);
        drive_->drive(0.0f, NAVIGATION_SPEED_MM_S, omega);
      }
      break;
    }
    case robot::MazeState::MAZE_DECIDE_TURN:
      if (distance_->leftHealthy() && distance_->readLeft() > TURN_CLEARANCE_MM) {
        beginTurn(90.0f);
        transition(robot::MazeState::MAZE_TURN_LEFT);
      } else if (distance_->frontHealthy() && distance_->readFront() > FRONT_STOP_MM) {
        const robot::RobotPose pose = odometry_ != nullptr ? odometry_->pose() : robot::RobotPose{0, 0, targetYaw_};
        moveStartX_ = pose.xMm;
        moveStartY_ = pose.yMm;
        targetYaw_ = imu_ != nullptr ? imu_->getYaw() : targetYaw_;
        transition(robot::MazeState::MAZE_FORWARD);
      } else if (distance_->rightHealthy() && distance_->readRight() > TURN_CLEARANCE_MM) {
        beginTurn(-90.0f);
        transition(robot::MazeState::MAZE_TURN_RIGHT);
      } else {
        beginTurn(180.0f);
        transition(robot::MazeState::MAZE_TURN_BACK);
      }
      break;
    case robot::MazeState::MAZE_TURN_LEFT:
    case robot::MazeState::MAZE_TURN_RIGHT:
    case robot::MazeState::MAZE_TURN_BACK: {
      if (imu_ == nullptr || !imu_->healthy()) {
        drive_->stop();
        transition(robot::MazeState::MAZE_RECOVERY);
        break;
      }
      float error = targetYaw_ - (imu_ != nullptr ? imu_->getYaw() : targetYaw_);
      if (error > 180.0f) error -= 360.0f;
      if (error < -180.0f) error += 360.0f;
      if (fabs(error) < MAZE_TURN_TOLERANCE_DEG) {
        drive_->stop();
        if (returning_) {
          transition(robot::MazeState::MAZE_RETURN);
          returnPhase_ = 2;
          const robot::RobotPose pose = odometry_ != nullptr ? odometry_->pose() : robot::RobotPose{0, 0, 0};
          moveStartX_ = pose.xMm;
          moveStartY_ = pose.yMm;
        } else {
          transition(robot::MazeState::MAZE_FIND_PATH);
        }
      } else if (millis() - stateStartedMs_ > MAZE_TURN_TIMEOUT_MS) {
        drive_->stop();
        transition(robot::MazeState::MAZE_RECOVERY);
      } else {
        const float omega = headingPid_.update(-error);
        drive_->drive(0.0f, 0.0f, omega);
      }
      break;
    }
    case robot::MazeState::MAZE_COLOR_CHECK:
      transition(robot::MazeState::MAZE_FIND_PATH);
      break;
    case robot::MazeState::MAZE_RECOVERY:
      drive_->stop();
      currentState_ = robot::PistaAState::A_CHECKPOINT;
      break;
    case robot::MazeState::MAZE_FINISH:
    case robot::MazeState::MAZE_INIT:
      drive_->stop();
      break;
    case robot::MazeState::MAZE_RETURN:
      updateReturn();
      break;
  }
}

void MazeNavigator::reset() {
  currentState_ = robot::PistaAState::A_INIT;
  mazeState_ = robot::MazeState::MAZE_INIT;
  returning_ = false;
  returnPhase_ = 0;
  if (checkpoints_ != nullptr) checkpoints_->clearPath();
}

robot::PistaAState MazeNavigator::state() const {
  return currentState_;
}

void MazeNavigator::transition(robot::MazeState next) {
  mazeState_ = next;
  stateStartedMs_ = millis();
}

void MazeNavigator::beginTurn(float deltaDeg) {
  headingPid_.reset();
  targetYaw_ = imu_ != nullptr ? imu_->getYaw() + deltaDeg : deltaDeg;
  if (targetYaw_ >= 360.0f) targetYaw_ -= 360.0f;
  if (targetYaw_ < 0.0f) targetYaw_ += 360.0f;
}

void MazeNavigator::updateReturn() {
  if (checkpoints_ == nullptr || odometry_ == nullptr || imu_ == nullptr || !imu_->healthy()) {
    drive_->stop();
    currentState_ = robot::PistaAState::A_FINISH;
    mazeState_ = robot::MazeState::MAZE_FINISH;
    return;
  }
  if (returnPhase_ == 0) {
    if (!checkpoints_->popReturnDirection(returnDirection_)) {
      drive_->stop();
      currentState_ = robot::PistaAState::A_FINISH;
      mazeState_ = robot::MazeState::MAZE_FINISH;
      return;
    }
    returnTargetYaw_ = (returnDirection_ * 90.0f) + 180.0f;
    if (returnTargetYaw_ >= 360.0f) returnTargetYaw_ -= 360.0f;
    targetYaw_ = returnTargetYaw_;
    returnPhase_ = 1;
    headingPid_.reset();
  }
  if (returnPhase_ == 1) {
    float error = targetYaw_ - imu_->getYaw();
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    if (fabs(error) > MAZE_TURN_TOLERANCE_DEG) {
      drive_->drive(0.0f, 0.0f, headingPid_.update(-error));
      return;
    }
    drive_->stop();
    const robot::RobotPose pose = odometry_->pose();
    moveStartX_ = pose.xMm;
    moveStartY_ = pose.yMm;
    returnPhase_ = 3;
    stateStartedMs_ = millis();
  }
  if (returnPhase_ == 3) {
    drive_->stop();
    if (millis() - stateStartedMs_ < 100) return;
    returnPhase_ = 2;
  }
  if (returnPhase_ == 2) {
    if (!distance_->frontHealthy() || distance_->readFront() <= FRONT_STOP_MM) {
      drive_->stop();
      checkpoints_->invalidatePath();
      currentState_ = robot::PistaAState::A_FINISH;
      mazeState_ = robot::MazeState::MAZE_FINISH;
      return;
    }
    const robot::RobotPose pose = odometry_->pose();
    const float dx = pose.xMm - moveStartX_;
    const float dy = pose.yMm - moveStartY_;
    if (sqrtf(dx * dx + dy * dy) >= MOVE_ONE_CELL_MM - ODOMETRY_CELL_TOLERANCE_MM) {
      drive_->stop();
      returnPhase_ = 0;
    } else {
      float error = targetYaw_ - imu_->getYaw();
      if (error > 180.0f) error -= 360.0f;
      if (error < -180.0f) error += 360.0f;
      drive_->drive(0.0f, NAVIGATION_SPEED_MM_S, headingPid_.update(-error));
    }
  }
}
