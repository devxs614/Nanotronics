#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

class PIDController {
public:
  PIDController();

  void setTunings(float kp, float ki, float kd);
  void setSampleTimeMs(uint16_t ms);
  void setOutputLimits(float minOut, float maxOut);
  void setSetpoint(float setpoint);
  void setMeasurement(float measurement);
  void reset();
  void enable(bool enabled);
  float update(float measurement);
  float getOutput() const;

private:
  float kp_; 
  float ki_;
  float kd_;
  float setpoint_;
  float lastMeasurement_;
  float integral_;
  float output_;
  float minOutput_;
  float maxOutput_;
  uint32_t lastUpdateMs_;
  uint16_t sampleTimeMs_;
  bool enabled_;
};

#endif
