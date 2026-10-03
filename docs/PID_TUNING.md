# PID tuning

Use an iterative approach: start with conservative gains and increase only after stable behavior is observed.

## Recommended first values
- KP: 0.35
- KI: 0.10
- KD: 0.02

## Process
1. Run the drivetrain test with low PWM values.
2. Measure actual RPM against command.
3. Increase proportional gain until the system responds with moderate overshoot.
4. Add a small integral term to reduce steady error.
5. Use derivative only to damp oscillation.
6. Re-check heading PID and wheel PID separately.

## Safety rules
- Do not run motors at full power on the first test.
- Tune one wheel at a time.
- Stop immediately if the robot starts oscillating or veers unpredictably.
