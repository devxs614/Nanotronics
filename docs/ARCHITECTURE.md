# Architecture

The system is divided into clear subsystems so that each module is testable independently.

## Core layers
1. Hardware abstraction: motor, encoder, sensor, gripper, display
2. Estimation: IMU + odometry + sensor fusion
3. Navigation: maze solving, line avoidance, tile following
4. Competition logic: state machine, recovery, checkpoints, competition mode
5. Diagnostics and calibration: hardware validation and tuning

## Hardware responsibility map
- `MotorController`: TB6612 outputs and acceleration limits
- `EncoderManager`: counts wheel ticks and computes speed
- `PIDController`: wheel speed regulation and heading loop
- `MecanumDrive`: kinematic mapping between robot frame and wheel commands
- `IMUManager`: yaw, pitch, roll and heading integration
- `DistanceSensors`: VL53L0X front, left, right
- `ColorSensor`: TCS34725 classification
- `LineSensor`: QTR-8A white-line detection
- `GripperController`: SG90 servo for the ball gripper
- `DisplayManager`: OLED status, sensors, modes, errors
- `RobotStateMachine`: high-level mode execution
- `MazeNavigator`, `BallHandler`, `TileNavigator`, `LineAvoidanceController`: track-specific logic
- `RecoveryManager` and `CheckpointManager`: reset-safe execution management

## Why this structure
This keeps each subsystem reasonably small, simplifies debugging, allows isolated tests, and avoids dead-end monolithic sketches. It is suitable for engineering students to review and update.
