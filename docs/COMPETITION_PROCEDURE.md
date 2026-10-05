# Competition procedure

## Before the round
- Verify drivetrain direction
- Validate encoder signs
- Check IMU yaw and pitch
- Confirm gripper servo limits
- Verify OLED and sensors are alive
- Make sure `COMPETITION_MODE` is enabled and the robot is not dependent on serial
- Verify `ACTIVE_TRACK` and `START_MODE` in `src/competition_config.h`
- Confirm the optional camera UART is disabled unless a camera is physically installed

## During calibration
- Keep the robot still for gyro-bias calibration
- Move the QTR array over both green floor and white line surfaces during the calibration window
- Use the calibration menu to align motors, encoders, and sensors
- Save corrections in `src/config.h`
- Re-run hardware tests before the start signal

## Start
- Confirm startup sequence: boot -> self test -> sensors ready -> calibration -> arm -> start
- Keep navigation autonomous and deterministic

## Recovery
- If the robot loses progress, trigger the checkpoint-based reset logic
- Re-enter from the latest safe checkpoint without a fixed map or external control
- Set the preselected checkpoint start mode before a competition run; EEPROM retains the last checkpoint and LOP count. Resume is accepted only when the saved checkpoint exists and matches the configured track.

## Finish
- Stop motors safely and complete the final state transition
- Record any observed issues and revise calibration for the next round
