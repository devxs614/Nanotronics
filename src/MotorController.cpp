#include "MotorController.h"

void MotorController::begin() {
  motorPins_[0][0] = MOTOR1_PWM_PIN; motorPins_[0][1] = MOTOR1_AIN1; motorPins_[0][2] = MOTOR1_AIN2;
  motorPins_[1][0] = MOTOR2_PWM_PIN; motorPins_[1][1] = MOTOR2_BIN1; motorPins_[1][2] = MOTOR2_BIN2;
  motorPins_[2][0] = MOTOR3_PWM_PIN; motorPins_[2][1] = MOTOR3_AIN1; motorPins_[2][2] = MOTOR3_AIN2;
  motorPins_[3][0] = MOTOR4_PWM_PIN; motorPins_[3][1] = MOTOR4_BIN1; motorPins_[3][2] = MOTOR4_BIN2;

  for (uint8_t motor = 0; motor < 4; ++motor) {
    pinMode(motorPins_[motor][0], OUTPUT);
    pinMode(motorPins_[motor][1], OUTPUT);
    pinMode(motorPins_[motor][2], OUTPUT);
    targetPWM[motor] = 0;
    currentPWM[motor] = 0;
  }
  stopAll();
}

int16_t MotorController::clampPWM(int16_t value) const {
  if (value > MOTOR_PWM_MAX) return MOTOR_PWM_MAX;
  if (value < -MOTOR_PWM_MAX) return -MOTOR_PWM_MAX;
  return value;
}

void MotorController::setMotorPWM(uint8_t motor, int16_t pwm) {
  if (motor >= 4) return;
  targetPWM[motor] = clampPWM(pwm);
}

void MotorController::setMotorDirection(uint8_t motor, MotorDirection direction) {
  if (motor >= 4) return;

  switch (direction) {
    case MotorDirection::FORWARD:
      digitalWrite(motorPins_[motor][1], HIGH);
      digitalWrite(motorPins_[motor][2], LOW);
      break;
    case MotorDirection::REVERSE:
      digitalWrite(motorPins_[motor][1], LOW);
      digitalWrite(motorPins_[motor][2], HIGH);
      break;
    case MotorDirection::BRAKE:
      digitalWrite(motorPins_[motor][1], HIGH);
      digitalWrite(motorPins_[motor][2], HIGH);
      break;
    case MotorDirection::COAST:
    default:
      digitalWrite(motorPins_[motor][1], LOW);
      digitalWrite(motorPins_[motor][2], LOW);
      break;
  }
}

void MotorController::brakeMotor(uint8_t motor) {
  setMotorDirection(motor, MotorDirection::BRAKE);
  targetPWM[motor] = 0;
}

void MotorController::coastMotor(uint8_t motor) {
  setMotorDirection(motor, MotorDirection::COAST);
  targetPWM[motor] = 0;
}

void MotorController::stopMotor(uint8_t motor) {
  setMotorPWM(motor, 0);
  setMotorDirection(motor, MotorDirection::COAST);
}

void MotorController::stopAll() {
  for (uint8_t motor = 0; motor < 4; ++motor) {
    targetPWM[motor] = 0;
    currentPWM[motor] = 0;
    analogWrite(motorPins_[motor][0], 0);
    digitalWrite(motorPins_[motor][1], LOW);
    digitalWrite(motorPins_[motor][2], LOW);
  }
}

void MotorController::update() {
  for (uint8_t motor = 0; motor < 4; ++motor) {
    int16_t delta = targetPWM[motor] - currentPWM[motor];
    int16_t step = 0;

    if (delta > 0) {
      step = (delta > ACCELERATION_LIMIT) ? static_cast<int16_t>(ACCELERATION_LIMIT) : delta;
      currentPWM[motor] += step;
    } else if (delta < 0) {
      step = ((-delta) > ACCELERATION_LIMIT) ? static_cast<int16_t>(ACCELERATION_LIMIT) : -delta;
      currentPWM[motor] -= step;
    }

    int16_t pwm = currentPWM[motor];
    if (pwm >= 0) {
      setMotorDirection(motor, MotorDirection::FORWARD);
      analogWrite(motorPins_[motor][0], (uint8_t)pwm);
    } else {
      setMotorDirection(motor, MotorDirection::REVERSE);
      analogWrite(motorPins_[motor][0], (uint8_t)(-pwm));
    }
  }
}
