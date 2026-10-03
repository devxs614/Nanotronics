# Troubleshooting

## Common issues
- Motor spins opposite direction: check inversion flags in `config.h`
- Encoder counts drift: inspect pin wiring and input pullups
- IMU yaw drifts: recalibrate and add heading corrections during turns
- VL53 readings are noisy: average or median filter nearby values
- Color classification fails: recalibrate white balance and thresholds
- OLED not responding: verify I2C bus and 0x3C address

## Safety
- All sensors should be checked individually before autonomous navigation is enabled
- Never assume a sensor is healthy just because the sketch compiles
- Treat `Serial` as a debug tool, not as a required navigation path
