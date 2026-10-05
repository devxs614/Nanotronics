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
- `EncoderManager`: quadrature counting and speed estimation; PCINT for D10-D15 and native external interrupts for D18-D19
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

The three VL53L0X modules are initialized one at a time with XSHUT so their default I2C addresses do not collide. D28 switches TCS34725 power off during the VL53 address assignment, then the color module restores power. The optional ArUco UART uses Serial2 (D16/D17), not Serial1, because D18/D19 are encoder inputs.

`src/config.h` is the single source of truth for hardware pins and physical parameters. `lib/constantes/constantes.h` only forwards to it.

## Why this structure
This keeps each subsystem reasonably small, simplifies debugging, allows isolated tests, and avoids dead-end monolithic sketches. It is suitable for engineering students to review and update.
