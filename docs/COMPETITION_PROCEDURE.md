# Competition procedure

## Before the round
- Verify drivetrain direction
- Validate encoder signs
- Check IMU yaw and pitch
- Confirm gripper servo limits
- Verify OLED and sensors are alive
- Make sure `COMPETITION_MODE` is enabled and the robot is not dependent on serial

## During calibration
- Use the calibration menu to align motors and sensors
- Save corrections in `src/config.h`
- Re-run hardware tests before the start signal

## Start
- Confirm startup sequence: boot -> self test -> sensors ready -> calibration -> arm -> start
- Keep navigation autonomous and deterministic

## Recovery
- If the robot loses progress, trigger the checkpoint-based reset logic
- Re-enter from the latest safe checkpoint without a fixed map or external control

## Finish
- Stop motors safely and complete the final state transition
- Record any observed issues and revise calibration for the next round
