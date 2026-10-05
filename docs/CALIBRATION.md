# Calibration guide

This robot relies heavily on calibration. The system is designed to keep all critical values in a single `src/config.h` file.

## 1. Motor direction
- Validate `MOTOR1_INVERTED`, `MOTOR2_INVERTED`, `MOTOR3_INVERTED`, `MOTOR4_INVERTED`
- Run the drivetrain test and confirm each wheel spins in the expected direction

## 2. Encoder direction
- Verify that positive wheel motion produces the expected tick sign in `EncoderManager`
- Correct `ENCODER*_INVERTED` if necessary
- M1-M3 use PCINT; M4 uses D18/D19 native external interrupts
- Measure all four output-shaft tick counts independently; do not assume they are identical

## 3. Wheel diameter and track dimensions
- Measure wheel diameter and robot geometry
- Update `WHEEL_DIAMETER_MM`, `WHEEL_BASE_MM`, `TRACK_WIDTH_MM`, `ROBOT_WIDTH_MM`, `ROBOT_LENGTH_MM`

## 4. PID tuning
- Start with moderate gains, then tune on the actual robot
- Use the `PIDController` test path and observe overshoot and steady-state error

## 5. IMU calibration
- Keep the robot motionless during calibration
- Run `IMUManager::calibrate()`
- Re-check yaw drift during slow turns

## 6. VL53L0X calibration
- Validate sensor distances against known target distances
- Confirm the XSHUT sequence is correct and addresses are stable

## 7. Color calibration
- Place the sensor over each target tile color and record raw values
- Check the `COLOR_CLEAR_MIN`, `COLOR_SATURATION_MIN`, and color classification thresholds in `src/config.h`

## 8. QTR calibration
- Place the robot over white lines and normal surfaces
- Calibrate `lineCalibrationMin[]` and `lineCalibrationMax[]`

## 9. Servo calibration
- Run `calibrateGripper()` and tune `GRIPPER_OPEN_ANGLE`, `GRIPPER_CLOSE_ANGLE`, `GRIPPER_HOLD_ANGLE`

## 10. Final validation
- Run each hardware test independently
- Only then enable the autonomous competition state machine

`BALL_CAPTURE_DISTANCE_MM` (45 mm by default) is a provisional geometric threshold, not a verified possession sensor. Adjust it only after validating the robot perimeter and the ball position; the current hardware cannot directly measure whether the gripper has retained the ball.

When starting a new run, use `START_FROM_BEGINNING`. To resume after a reset, choose the appropriate checkpoint start mode; startup refuses the resume if that checkpoint is unavailable or belongs to a different track. Checkpoint pose and section LOP count are saved in Mega EEPROM; addresses are reserved by `CHECKPOINT_EEPROM_BASE` and `RECOVERY_EEPROM_BASE` in `src/config.h`. A section LOP is recorded after 12 seconds without at least 15 mm of odometric progress; after four reports, Pista B advances to its next section.
