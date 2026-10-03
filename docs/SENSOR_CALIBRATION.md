# Sensor calibration

This document is a checklist for sensor calibration before competition use.

## VL53L0X
- Confirm XSHUT sequence and I2C addresses
- Measure front-left-right distances against known targets
- Ensure no sensor is saturating or returning invalid values

## TCS34725
- Place over each target tile color
- Record raw R/G/B/C values
- Update thresholds carefully in `config.h`

## QTR-8A
- Calibrate dark and light surfaces
- Verify line thresholds before white-line avoidance mode is enabled

## IMU
- Place the robot stationary, run calibration, and inspect drift
- Validate yaw during controlled turns

## Servo
- Tune gripper open, close, and hold positions
- Confirm the ball is not over-compressed or under-held
