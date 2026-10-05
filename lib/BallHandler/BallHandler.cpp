#include "BallHandler.h"

void BallHandler::begin() {
  state_ = robot::BallHandlerState::BALL_SEARCH;
  ballState_ = robot::BallState::UNKNOWN;
  stateStartedMs_ = millis();
  alignPhaseStartedMs_ = 0;
  captureDistanceMm_ = 0;
  alignBaselineMm_ = 0;
  alignPhase_ = 0;
  objectValidated_ = false;
  secureMotionStarted_ = false;
  secureStartX_ = secureStartY_ = 0.0f;
}

void BallHandler::configure(DistanceSensors* distance, GripperController* gripper,
                            MecanumDrive* drive, Odometry* odometry) {
  distance_ = distance;
  gripper_ = gripper;
  drive_ = drive;
  odometry_ = odometry;
}

void BallHandler::update() {
  if (distance_ == nullptr || gripper_ == nullptr || drive_ == nullptr) return;
  const uint16_t front = distance_->readFront();
  const bool frontValid = distance_->frontHealthy();

  switch (state_) {
    case robot::BallHandlerState::BALL_SEARCH:
      ballState_ = robot::BallState::NOT_CAPTURED;
      if (!frontValid) {
        drive_->stop();
      } else if (front <= BALL_SEARCH_DETECTION_MM) {
        captureDistanceMm_ = front;
        transition(robot::BallHandlerState::BALL_APPROACH);
      } else {
        drive_->driveForward(BALL_SEARCH_SPEED_MM_S);
      }
      break;
    case robot::BallHandlerState::BALL_APPROACH:
      if (!frontValid) {
        drive_->stop();
        ballState_ = robot::BallState::BALL_UNCERTAIN;
        transition(robot::BallHandlerState::BALL_RECOVER);
      } else if (objectValidated_ && front <= BALL_CAPTURE_DISTANCE_MM) {
        captureDistanceMm_ = front;
        drive_->stop();
        transition(robot::BallHandlerState::BALL_GRAB);
      } else if (!objectValidated_ && front <= BALL_APPROACH_MM) {
        drive_->stop();
        alignPhase_ = 0;
        transition(robot::BallHandlerState::BALL_ALIGN);
      } else {
        captureDistanceMm_ = front;
        drive_->driveForward(front > BALL_APPROACH_MM
          ? BALL_FAST_APPROACH_SPEED_MM_S : BALL_SLOW_APPROACH_SPEED_MM_S);
      }
      break;
    case robot::BallHandlerState::BALL_ALIGN:
      if (alignPhase_ == 0) {
        alignBaselineMm_ = frontValid ? front : 0;
        alignPhase_ = 1;
        alignPhaseStartedMs_ = millis();
        drive_->strafeLeft(BALL_ALIGN_SWEEP_SPEED_MM_S);
      } else if (alignPhase_ == 1 && millis() - alignPhaseStartedMs_ >= BALL_ALIGN_SWEEP_MS) {
        drive_->stop();
        alignPhase_ = 2;
        alignPhaseStartedMs_ = millis();
      } else if (alignPhase_ == 2 && millis() - alignPhaseStartedMs_ >= BALL_ALIGN_SETTLE_MS) {
        const uint16_t rangeChange = frontValid
          ? (front > alignBaselineMm_ ? front - alignBaselineMm_ : alignBaselineMm_ - front)
          : alignBaselineMm_;
        if (alignBaselineMm_ != 0 && rangeChange >= BALL_OBJECT_RANGE_CHANGE_MM) {
          objectValidated_ = true;
          captureDistanceMm_ = frontValid ? front : alignBaselineMm_;
          transition(robot::BallHandlerState::BALL_APPROACH);
        } else {
          transition(robot::BallHandlerState::BALL_RECOVER);
        }
      }
      break;
    case robot::BallHandlerState::BALL_GRAB:
      gripper_->close();
      ballState_ = robot::BallState::CAPTURING;
      if (millis() - stateStartedMs_ >= BALL_GRIPPER_SETTLE_MS) {
        transition(robot::BallHandlerState::BALL_VERIFY);
      }
      break;
    case robot::BallHandlerState::BALL_VERIFY:
      drive_->stop();
      if (captureDistanceMm_ > 0 && captureDistanceMm_ <= BALL_CAPTURE_DISTANCE_MM) {
        ballState_ = robot::BallState::CAPTURED;
        transition(robot::BallHandlerState::BALL_SECURE);
      } else if (millis() - stateStartedMs_ >= BALL_VERIFY_TIMEOUT_MS) {
        ballState_ = robot::BallState::BALL_UNCERTAIN;
        transition(robot::BallHandlerState::BALL_RECOVER);
      }
      break;
    case robot::BallHandlerState::BALL_SECURE:
      gripper_->hold();
      if (odometry_ == nullptr) {
        drive_->stop();
        ballState_ = robot::BallState::BALL_UNCERTAIN;
        transition(robot::BallHandlerState::BALL_RECOVER);
        break;
      }
      if (!secureMotionStarted_) {
        const robot::RobotPose pose = odometry_->pose();
        secureStartX_ = pose.xMm;
        secureStartY_ = pose.yMm;
        secureMotionStarted_ = true;
      }
      {
        const robot::RobotPose pose = odometry_->pose();
        const float dx = pose.xMm - secureStartX_;
        const float dy = pose.yMm - secureStartY_;
        if (sqrtf(dx * dx + dy * dy) >= MOVE_ONE_CELL_MM - ODOMETRY_CELL_TOLERANCE_MM) {
          drive_->stop();
          ballState_ = robot::BallState::SECURED;
        } else if (millis() - stateStartedMs_ >= BALL_SECURE_TIMEOUT_MS) {
          drive_->stop();
          ballState_ = robot::BallState::BALL_UNCERTAIN;
          transition(robot::BallHandlerState::BALL_RECOVER);
        } else {
          drive_->driveBackward(BALL_SECURE_SPEED_MM_S);
        }
      }
      break;
    case robot::BallHandlerState::BALL_LOST:
      ballState_ = robot::BallState::LOST;
      drive_->stop();
      break;
    case robot::BallHandlerState::BALL_RECOVER:
      gripper_->open();
      if (millis() - stateStartedMs_ < BALL_RECOVERY_TURN_MS) {
        drive_->rotateCCW(0.35f);
      } else {
        drive_->stop();
      }
      if (millis() - stateStartedMs_ >= BALL_RECOVERY_DURATION_MS) {
        ballState_ = robot::BallState::NOT_CAPTURED;
        objectValidated_ = false;
        transition(robot::BallHandlerState::BALL_SEARCH);
      }
      break;
    case robot::BallHandlerState::BALL_RELEASE:
      drive_->stop();
      gripper_->open();
      ballState_ = robot::BallState::NOT_CAPTURED;
      transition(robot::BallHandlerState::BALL_SEARCH);
      break;
  }
}

bool BallHandler::hasBall() const {
  return ballState_ == robot::BallState::SECURED;
}

void BallHandler::setBallState(robot::BallState state) {
  ballState_ = state;
}

robot::BallHandlerState BallHandler::state() const { return state_; }
robot::BallState BallHandler::ballState() const { return ballState_; }

void BallHandler::release() {
  transition(robot::BallHandlerState::BALL_RELEASE);
}

void BallHandler::transition(robot::BallHandlerState next) {
  state_ = next;
  stateStartedMs_ = millis();
  if (next == robot::BallHandlerState::BALL_SECURE) secureMotionStarted_ = false;
}
