#include "RobotStateMachine.h"

void RobotStateMachine::begin() {
  currentMode_ = robot::RobotMode::STARTUP;
  pistaBState_ = robot::PistaBState::B_INIT;
  stateStartedMs_ = millis();
  sectionStartX_ = sectionStartY_ = 0.0f;
  progressReferenceX_ = progressReferenceY_ = 0.0f;
  lastProgressMs_ = 0;
  progressSection_ = 0;
  progressMonitorInitialized_ = false;
}

void RobotStateMachine::configure(robot::ActiveTrack track, MazeNavigator* maze, BallHandler* ball,
                                  LineAvoidanceController* lineAvoidance, TileNavigator* tiles,
                                  CheckpointManager* checkpoints, RecoveryManager* recovery,
                                  Odometry* odometry, MecanumDrive* drive, DisplayManager* display) {
  activeTrack_ = track;
  maze_ = maze;
  ball_ = ball;
  lineAvoidance_ = lineAvoidance;
  tiles_ = tiles;
  checkpoints_ = checkpoints;
  recovery_ = recovery;
  odometry_ = odometry;
  drive_ = drive;
  display_ = display;
}

void RobotStateMachine::update() {
  switch (currentMode_) {
    case robot::RobotMode::STARTUP:
      currentMode_ = robot::RobotMode::SELF_TEST;
      stateStartedMs_ = millis();
      if (display_ != nullptr) display_->showStatus("SELF TEST", "SENSORS");
      break;
    case robot::RobotMode::SELF_TEST:
      if (millis() - stateStartedMs_ >= 300) {
        if (COMPETITION_MODE) {
          currentMode_ = robot::RobotMode::CALIBRATION;
          stateStartedMs_ = millis();
          if (display_ != nullptr) display_->showStatus("CALIBRATION", "2 MINUTES");
        } else {
          currentMode_ = robot::RobotMode::WAIT_FOR_START;
          if (display_ != nullptr) display_->showStatus("READY", "WAIT START");
        }
      }
      break;
    case robot::RobotMode::WAIT_FOR_START:
      if (START_IMMEDIATELY || COMPETITION_MODE) startSelectedTrack();
      break;
    case robot::RobotMode::PISTA_A:
      if (maze_ != nullptr) {
        maze_->update();
        if (maze_->state() == robot::PistaAState::A_FINISH) {
          currentMode_ = robot::RobotMode::FINISHED;
        }
      }
      break;
    case robot::RobotMode::PISTA_B:
      updatePistaB();
      updateProgressMonitor();
      break;
    case robot::RobotMode::RECOVERY:
      if (drive_ != nullptr) drive_->stop();
      if (recovery_ != nullptr) recovery_->resetToCheckpoint();
      currentMode_ = activeTrack_ == robot::ActiveTrack::PISTA_A
        ? robot::RobotMode::PISTA_A : robot::RobotMode::PISTA_B;
      break;
    case robot::RobotMode::FINISHED:
      if (drive_ != nullptr) drive_->stop();
      if (display_ != nullptr) display_->showStatus("FINISHED", "STOPPED");
      break;
    case robot::RobotMode::ERROR:
      if (drive_ != nullptr) drive_->stop();
      if (display_ != nullptr) display_->showError("ROBOT ERROR");
      break;
    case robot::RobotMode::CALIBRATION:
      if (millis() - stateStartedMs_ >= CALIBRATION_DURATION_MS) startSelectedTrack();
      break;
  }
}

robot::RobotMode RobotStateMachine::getCurrentMode() const {
  return currentMode_;
}

robot::PistaBState RobotStateMachine::getPistaBState() const {
  return pistaBState_;
}

void RobotStateMachine::setMode(robot::RobotMode mode) {
  currentMode_ = mode;
  stateStartedMs_ = millis();
}

void RobotStateMachine::startSelectedTrack() {
  if (START_MODE == robot::StartMode::START_FROM_BEGINNING) {
    if (recovery_ != nullptr) recovery_->resetLopCounters();
    if (checkpoints_ != nullptr) checkpoints_->reset();
  } else {
    const uint8_t requiredCheckpoint =
      START_MODE == robot::StartMode::START_FROM_CHECKPOINT_1 ? 1 : 2;
    if (activeTrack_ != robot::ActiveTrack::PISTA_B ||
        checkpoints_ == nullptr || recovery_ == nullptr ||
        checkpoints_->activeTrack() != activeTrack_ ||
        checkpoints_->currentCheckpoint() < requiredCheckpoint) {
      currentMode_ = robot::RobotMode::ERROR;
      if (drive_ != nullptr) drive_->stop();
      if (display_ != nullptr) display_->showError("CHECKPOINT INVALID");
      return;
    }
    recovery_->resetToCheckpoint();
  }
  if (activeTrack_ == robot::ActiveTrack::PISTA_A) {
    if (maze_ != nullptr) maze_->reset();
    currentMode_ = robot::RobotMode::PISTA_A;
    if (display_ != nullptr) display_->showStatus("PISTA A", "MAZE");
    return;
  }

  if (START_MODE == robot::StartMode::START_FROM_CHECKPOINT_1) {
    pistaBState_ = robot::PistaBState::B_SECTION2;
  } else if (START_MODE == robot::StartMode::START_FROM_CHECKPOINT_2) {
    pistaBState_ = robot::PistaBState::B_SECTION3;
  } else {
    pistaBState_ = robot::PistaBState::B_INIT;
  }
  currentMode_ = robot::RobotMode::PISTA_B;
  stateStartedMs_ = millis();
  progressMonitorInitialized_ = false;
  if (display_ != nullptr) display_->showStatus("PISTA B", "BALL");
}

void RobotStateMachine::updateProgressMonitor() {
  if (odometry_ == nullptr) return;

  uint8_t section;
  switch (pistaBState_) {
    case robot::PistaBState::B_FIND_BALL:
      section = 1;
      break;
    case robot::PistaBState::B_AVOID_LINES:
      section = 2;
      break;
    case robot::PistaBState::B_MOVE_TO_NEXT_TILE:
      section = 3;
      break;
    default:
      progressMonitorInitialized_ = false;
      return;
  }

  const robot::RobotPose pose = odometry_->pose();
  if (!progressMonitorInitialized_ || section != progressSection_) {
    progressReferenceX_ = pose.xMm;
    progressReferenceY_ = pose.yMm;
    progressSection_ = section;
    lastProgressMs_ = millis();
    progressMonitorInitialized_ = true;
    return;
  }

  const float dx = pose.xMm - progressReferenceX_;
  const float dy = pose.yMm - progressReferenceY_;
  if (sqrtf(dx * dx + dy * dy) >= LOP_MIN_PROGRESS_MM) {
    progressReferenceX_ = pose.xMm;
    progressReferenceY_ = pose.yMm;
    lastProgressMs_ = millis();
  } else if (millis() - lastProgressMs_ >= LOP_NO_PROGRESS_TIMEOUT_MS) {
    lastProgressMs_ = millis();
    reportLackOfProgress();
  }
}

void RobotStateMachine::updatePistaB() {
  if (display_ != nullptr) display_->setMode("PISTA B");
  switch (pistaBState_) {
    case robot::PistaBState::B_INIT:
      pistaBState_ = robot::PistaBState::B_FIND_BALL;
      stateStartedMs_ = millis();
      break;
    case robot::PistaBState::B_SECTION1:
    case robot::PistaBState::B_FIND_BALL:
    case robot::PistaBState::B_GRAB_BALL:
      if (ball_ == nullptr) {
        pistaBState_ = robot::PistaBState::B_ERROR;
        break;
      }
      ball_->update();
      if (ball_->hasBall()) {
        if (checkpoints_ != nullptr) checkpoints_->registerCheckpoint(1, 1, activeTrack_);
        if (odometry_ != nullptr) {
          sectionStartX_ = odometry_->x();
          sectionStartY_ = odometry_->y();
        }
        pistaBState_ = robot::PistaBState::B_SECTION2;
        if (display_ != nullptr) display_->showStatus("PISTA B", "CHECKPOINT 1");
      }
      break;
    case robot::PistaBState::B_TO_CHECKPOINT1:
      pistaBState_ = robot::PistaBState::B_SECTION2;
      break;
    case robot::PistaBState::B_SECTION2:
      if (lineAvoidance_ == nullptr || odometry_ == nullptr) {
        pistaBState_ = robot::PistaBState::B_ERROR;
        break;
      }
      pistaBState_ = robot::PistaBState::B_AVOID_LINES;
      sectionStartX_ = odometry_->x();
      sectionStartY_ = odometry_->y();
      break;
    case robot::PistaBState::B_AVOID_LINES: {
      lineAvoidance_->update();
      if (odometry_ != nullptr) {
        const float dx = odometry_->x() - sectionStartX_;
        const float dy = odometry_->y() - sectionStartY_;
        if (sqrtf(dx * dx + dy * dy) >= PISTA_B_SECTION2_CELLS * MOVE_ONE_CELL_MM) {
          if (drive_ != nullptr) drive_->stop();
          if (checkpoints_ != nullptr) checkpoints_->registerCheckpoint(2, 2, activeTrack_);
          pistaBState_ = robot::PistaBState::B_SECTION3;
        }
      }
      break;
    }
    case robot::PistaBState::B_TO_CHECKPOINT2:
      pistaBState_ = robot::PistaBState::B_SECTION3;
      break;
    case robot::PistaBState::B_SECTION3:
      if (tiles_ != nullptr) {
        tiles_->reset();
        pistaBState_ = robot::PistaBState::B_READ_TILE;
      } else {
        pistaBState_ = robot::PistaBState::B_ERROR;
      }
      break;
    case robot::PistaBState::B_READ_TILE:
    case robot::PistaBState::B_MOVE_TO_NEXT_TILE:
      if (tiles_ == nullptr) {
        pistaBState_ = robot::PistaBState::B_ERROR;
        break;
      }
      tiles_->update();
      pistaBState_ = tiles_->moving()
        ? robot::PistaBState::B_MOVE_TO_NEXT_TILE : robot::PistaBState::B_READ_TILE;
      if (tiles_->finished()) pistaBState_ = robot::PistaBState::B_FINISH;
      break;
    case robot::PistaBState::B_FINISH:
      currentMode_ = robot::RobotMode::FINISHED;
      if (drive_ != nullptr) drive_->stop();
      break;
    case robot::PistaBState::B_ERROR:
      currentMode_ = robot::RobotMode::ERROR;
      break;
  }
}

void RobotStateMachine::reportLackOfProgress() {
  if (recovery_ == nullptr) return;
  const uint8_t state = static_cast<uint8_t>(pistaBState_);
  const uint8_t section = state <= static_cast<uint8_t>(robot::PistaBState::B_GRAB_BALL) ? 1
    : (state <= static_cast<uint8_t>(robot::PistaBState::B_AVOID_LINES) ? 2 : 3);
  if (recovery_->reportLackOfProgress(section)) {
    currentMode_ = robot::RobotMode::RECOVERY;
    if (drive_ != nullptr) drive_->stop();
    if (activeTrack_ == robot::ActiveTrack::PISTA_B) {
      pistaBState_ = section == 1 ? robot::PistaBState::B_SECTION2
        : (section == 2 ? robot::PistaBState::B_SECTION3 : robot::PistaBState::B_FINISH);
      recovery_->requestForcedTransition();
    }
  }
}
