# Calibration guide

This robot relies heavily on calibration. The system is designed to keep all critical values in a single `src/config.h` file.

## 1. Motor direction
- Validate `MOTOR1_INVERTED`, `MOTOR2_INVERTED`, `MOTOR3_INVERTED`, `MOTOR4_INVERTED`
- Run the drivetrain test and confirm each wheel spins in the expected direction

## 2. Encoder direction
- Verify that positive wheel motion produces the expected tick sign in `EncoderManager`
- Correct `ENCODER*_INVERTED` if necessary

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
- Update thresholds in `config.h`

## 8. QTR calibration
- Place the robot over white lines and normal surfaces
- Calibrate `lineCalibrationMin[]` and `lineCalibrationMax[]`

## 9. Servo calibration
- Run `calibrateGripper()` and tune `GRIPPER_OPEN_ANGLE`, `GRIPPER_CLOSE_ANGLE`, `GRIPPER_HOLD_ANGLE`

## 10. Final validation
- Run each hardware test independently
- Only then enable the autonomous competition state machine
