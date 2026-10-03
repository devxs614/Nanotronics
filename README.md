# RoBorregos Candidates 2026 Robot

This project is a modular Arduino Mega 2560 autonomous robot framework for the RoBorregos Candidates 2026 competition. The code is organized around a robust control stack for motor control, encoders, odometry, sensing, navigation, state management, and competition-aware recovery.

## Hardware target
- Microcontroller: Arduino Mega 2560 CH340
- Motor driver: TB6612FNG x2
- Drive: 4-wheel mecanum
- Encoders: four wheel encoders on D10-D17
- Sensors: VL53L0X x3, TCS34725, QTR-8A, MPU6050, SG90 gripper servo, OLED

## Important notes
- This project is a professional scaffold and framework, not a black-box single-file sketch.
- Physical calibration values are intentionally centralized in `src/config.h`.
- Any unknown physical parameter is documented as provisional and should be measured experimentally.
- The robot is built for safe autonomous operation and does not depend on external communications during a run.

## Folder layout
- `src/` — Arduino source and modular classes
- `docs/` — technical notes and calibration procedures
- `test/` — hardware test skeletons

## Build
1. Install PlatformIO.
2. In this folder run:
   `pio run -e megaatmega2560`
3. Upload with:
   `pio run -e megaatmega2560 -t upload`

## First hardware tests
1. `MotorController` and drivetrain sanity check
2. `EncoderManager` validation
3. `IMUManager` orientation test
4. `DistanceSensors` VL53L0X validation
5. `ColorSensor` calibration and classification
6. `LineSensor` QTR detection
7. `GripperController` servo motion
8. `DisplayManager` OLED smoke test

## Competition safety
- Do not start motors at maximum power during boot.
- Use `EMERGENCY_STOP` path and `stopAllMotors()` when faults occur.
- Debug serial is for development; navigation must remain autonomous without laptop/serial.

## Current status
The project contains the architecture, modular APIs, and real C++ stubs for the target hardware. Physical tuning and calibration remain the final hardware-validation step before competition use.
