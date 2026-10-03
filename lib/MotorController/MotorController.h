#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

class MotorController {
public:
  enum class MotorDirection {
    FORWARD,
    REVERSE,
    BRAKE,
    COAST
  };

  void begin();
  void setMotorPWM(uint8_t motor, int16_t pwm);
  void setMotorDirection(uint8_t motor, MotorDirection direction);
  void brakeMotor(uint8_t motor);
  void coastMotor(uint8_t motor);
  void stopMotor(uint8_t motor);
  void stopAll();
  void update();

  int16_t targetPWM[4];
  int16_t currentPWM[4];

private:
  uint8_t motorPins_[4][3];

  int16_t clampPWM(int16_t value) const;
};

#endif
