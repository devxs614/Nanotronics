# Pinout

## Motor driver outputs
- M1 front-left: PWM = D6, AIN1 = D30, AIN2 = D31
- M2 front-right: PWM = D7, BIN1 = D32, BIN2 = D33
- M3 rear-left: PWM = D8, AIN1 = D35, AIN2 = D36
- M4 rear-right: PWM = D9, BIN1 = D37, BIN2 = D38

## Encoders
- M1 A = D10, B = D11
- M2 A = D12, B = D13
- M3 A = D14, B = D15
- M4 A = D18 (INT3), B = D19 (INT2)

D10-D15 use ATmega2560 pin-change interrupts (D10-D13 = PCINT4-PCINT7; D14 = PCINT10; D15 = PCINT9). D18-D19 use their native external interrupts; do not attach external interrupts to D10-D15. D16-D17 remain available for Serial2 if the optional external vision interface is enabled.

## I2C bus
- SDA = D20
- SCL = D21

## VL53L0X
- Front XSHUT = D24
- Right XSHUT = D22
- Left XSHUT = D26

## TCS34725 power
- TCS_POWER = D28

## Servo
- Gripper signal = D5

## QTR-8A
- A0-A7

## Display
- OLED I2C at 0x3C

## Sensor addresses
- TCS34725 = 0x29
- MPU6050 = 0x68
- VL53L0X front = 0x30
- VL53L0X right = 0x31
- VL53L0X left = 0x32
