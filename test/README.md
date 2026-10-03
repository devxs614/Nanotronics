# Hardware test plan

This directory contains hardware test skeletons for the robot subsystem validation flow.
Recommended order:
Blink
Digital pins
Motor 1..4 and drivetrain
Encoder checks
PID tuning
IMU validation
VL53L0X checks
TCS34725 classification
QTR-8A detection
Servo and gripper
OLED smoke test
Full integration test

Development log and troubleshooting

1. Actuator module and DC motors (TB6612FNG)

Problem detected

During the first motion tests with the TB6612FNG drivers, two of the four motors (specifically Motor 2 and Motor 3) did not respond to PWM commands sending correct direction signals, or their behavior was intermittent when changing the rotation direction.

Analysis and solution

Electrical verification: I used a multimeter to measure the output voltage at the A01/A02 and B01/B02 terminals of the driver, confirming that the voltage drop did not come from the power regulator, but from a floating logic signal.
Pin mapping and direct logic: I reviewed the pin assignment in the test/motor_test.cpp file, re-mapping the PWM outputs and direction lines (IN1/IN2) to pins with dedicated timer support on the Arduino Mega 2560.
Validation: With an isolated modular script in PlatformIO, I verified the individual behavior of each H-bridge in both directions at different duty cycles (0–255).

2. Odometry and interrupt diagnostics in quadrature encoders

Problem detected

When implementing the speed and pulse reading of the four quadrature encoders, motors M1, M2, and M3 registered precise counts through pin change interrupts (Pin Change Interrupts - PCINT). However, Motor 4 did not register any pulses when turning its axis.

ATmega2560 architecture analysis

When auditing the physical mapping of the pins assigned to Motor 4 (D16 and D17), I identified that they belong to Port H (PH0 and PH1) of the ATmega2560 microcontroller. Unlike ports B and J, Port H lacks hardware for PCINT vectors (PCINTx). Attempting to read these pins by polling inside the loop() function caused a massive loss of ticks when the motor spun at high revolutions.

Implemented solution

Physical re-routing: I reassigned the encoder reading lines of Motor 4 from pins D16/D17 to pins D18 (INT3) and D19 (INT2).
Leveraging native interrupts: I configured the reading using attachInterrupt(digitalPinToInterrupt(18), ISR_M4, RISING), utilizing the dedicated hardware interrupts of the chip instead of the port register.
Result: I achieved a deterministic high-frequency reading across all four odometry channels with zero pulse loss.

3. Line follower module handling (QTR-8A)

Problem detected

When updating the libraries to version QTRSensors v4.x from Pololu, the legacy syntax used in initial testing yielded compilation errors due to obsolete constants like QTR_EMITTERS_ON.

Implemented solution

Script modernization: I rewrote the test module test/qtr_test.cpp adapting it to the v4.x API.
Analog configuration: I explicitly defined the reading array with qtr.setTypeAnalog() and the mapping of the 8 analog channels of the Arduino Mega (A0 to A7).
Read optimization: I simplified the capture function to qtr.read(sensorValues), isolating raw analog readings for subsequent normalization using qtr.calibrate().

4. Address conflict on the I2C bus (3x VL53L0X + TCS34725)

The technical problem

The robot integrates three Time-of-Flight distance sensors (VL53L0X: Front, Right, Left) and an RGB color sensor (TCS34725). Upon performing an initial I2C scan, the terminal only detected a device at address 0x29.
Upon reviewing the component datasheets, I discovered that both the TCS34725 and the three VL53L0X sensors come factory-configured with the same default I2C address (0x29). With all of them powered simultaneously on the bus (SDA D20 / SCL D21), transmissions collided and the bus became completely locked.

Failed attempts and deep diagnosis

Simple software re-addressing: I attempted to execute the standard lox.begin(NEW_ADDRESS) routine by powering on the XSHUT pins one by one. However, the distance sensors kept failing with the message [FAIL].
Discovery of the conflict with the color sensor: Upon physically disconnecting the TCS34725 color sensor from the bus, all three VL53L0X distance sensors successfully changed their addresses to 0x30, 0x31, and 0x32 without any issues.
Root cause:
The Adafruit_VL53L0X library sends broadcast commands to address 0x29 during its initialization to order the address change.
Since the TCS34725 **has no XSHUT pin** to turn it off via software and its integrated pull-up resistors kept the bus impedance low, the color sensor responded and corrupted the packets intended for the VL53L0X sensors.

Dynamic power control via software

Since it was not possible to shut down the TCS34725 via software nor change its fixed factory address (0x29), I designed a power control and staged initialization strategy:
Hardware modification: I disconnected the power pin (VIN/VCC) of the TCS34725 sensor from the constant 5V line and connected it directly to digital pin D28 of the Arduino Mega.
XSHUT lines connection: I connected the reset lines of the VL53L0Xs to independent digital pins:
Front → D24
Right → D22
Left → D26
**Sequential startup algorithm in setup()**:
Step A: I set pin D28 to LOW state (0V, completely de-energizing the TCS34725) and set pins D22, D24, and D26 to LOW (holding all three VL53L0Xs in reset).
Step B: I set pin D24 to HIGH. I initialized the sensor at 0x29 and immediately assigned it the new address 0x30.
Step C: I set pin D22 to HIGH. I initialized the sensor at 0x29 and assigned it the address 0x31.
Step D: I set pin D26 to HIGH. I initialized the sensor at 0x29 and assigned it the address 0x32.
Step E: Having moved all three distance sensors to private addresses (0x30, 0x31, 0x32), address 0x29 was 100% free. At that moment, I toggled pin D28 to HIGH (5V) to power the TCS34725 and called tcs.begin(0x29, &Wire).

Result

The 4 I2C sensors coexist stably on the same data bus, allowing simultaneous real-time readings of distance (in mm) and color (RGB values) without any type of collision.

5. Development environment management and project structure (PlatformIO)

Challenge

Testing multiple peripherals individually without causing main function redefinition conflicts (setup() and loop()) in C/C++ or cluttering the main src/main.cpp folder.

Solution

I took advantage of the build_src_filter directive in the platformio.ini configuration file:
I created a test/ folder where I stored each diagnostic script in isolation (qtr_test.cpp, vl53_color_test.cpp, etc.).
I used the inclusion and exclusion syntax (+<../test/current_script.cpp> -<main.cpp>) to switch tests in a matter of seconds, drastically accelerating the development and compilation process prior to final integration.
* Creé una carpeta `test/` donde almacené cada script de diagnóstico de forma aislada (`qtr_test.cpp`, `vl53_color_test.cpp`, etc.).
* Utilicé la sintaxis de inclusión y exclusión (`+<../test/script_actual.cpp> -<main.cpp>`) para cambiar de prueba en cuestión de segundos, acelerando drásticamente el proceso de desarrollo y compilación antes de la integración final.
