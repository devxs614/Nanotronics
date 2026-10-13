#include "EncoderManager.h"

volatile int32_t encoderTicks[4] = {0, 0, 0, 0};

namespace {
float ticksPerOutputRev(uint8_t motor) {
  switch (motor) {
    case 0: return ENCODER_TICKS_PER_OUTPUT_REV_M1;
    case 1: return ENCODER_TICKS_PER_OUTPUT_REV_M2;
    case 2: return ENCODER_TICKS_PER_OUTPUT_REV_M3;
    default: return ENCODER_TICKS_PER_OUTPUT_REV_M4;
  }
}

// Flanco de subida en el canal A; el nivel del canal B define el sentido.
inline void countTick(uint8_t motor, uint8_t pinB, bool inverted) {
  const bool forward = (digitalRead(pinB) == HIGH) != inverted;
  encoderTicks[motor] += forward ? 1 : -1;
}

void isrEncoder1() { countTick(0, ENC1_PIN_B, ENCODER1_INVERTED); }
void isrEncoder2() { countTick(1, ENC2_PIN_B, ENCODER2_INVERTED); }
void isrEncoder3() { countTick(2, ENC3_PIN_B, ENCODER3_INVERTED); }
void isrEncoder4() { countTick(3, ENC4_PIN_B, ENCODER4_INVERTED); }

void attachEncoder(uint8_t index, uint8_t pinA, uint8_t pinB, void (*isr)()) {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  const int intNum = digitalPinToInterrupt(pinA);
  if (intNum == NOT_AN_INTERRUPT) {
    Serial.print(F("[ERROR ENCODER] Pin sin interrupcion (Enc"));
    Serial.print(index);
    Serial.print(F("): "));
    Serial.println(pinA);
    return;
  }
  attachInterrupt(intNum, isr, RISING);
}
}

void EncoderManager::begin() {
  for (uint8_t i = 0; i < 4; ++i) {
    sampleTicks_[i] = 0;
    rpm_[i] = 0.0f;
  }
  noInterrupts();
  for (uint8_t i = 0; i < 4; ++i) encoderTicks[i] = 0;
  interrupts();
  lastSampleMs_ = millis();

  attachEncoder(1, ENC1_PIN_A, ENC1_PIN_B, isrEncoder1);
  attachEncoder(2, ENC2_PIN_A, ENC2_PIN_B, isrEncoder2);
  attachEncoder(3, ENC3_PIN_A, ENC3_PIN_B, isrEncoder3);
  attachEncoder(4, ENC4_PIN_A, ENC4_PIN_B, isrEncoder4);
}

void EncoderManager::update() {
  const uint32_t now = millis();
  const uint32_t elapsedMs = now - lastSampleMs_;
  if (elapsedMs < ENCODER_RPM_SAMPLE_MS) return;

  int32_t currentTicks[4];
  noInterrupts();
  for (uint8_t motor = 0; motor < 4; ++motor) {
    currentTicks[motor] = encoderTicks[motor];
  }
  interrupts();

  for (uint8_t motor = 0; motor < 4; ++motor) {
    const float ticksPerRev = ticksPerOutputRev(motor);
    if (ticksPerRev > 0.0f) {
      const int32_t deltaTicks = currentTicks[motor] - sampleTicks_[motor];
      rpm_[motor] = (static_cast<float>(deltaTicks) * 60000.0f) /
                    (ticksPerRev * static_cast<float>(elapsedMs));
    } else {
      rpm_[motor] = 0.0f;
    }
    sampleTicks_[motor] = currentTicks[motor];
  }
  lastSampleMs_ = now;
}

int32_t EncoderManager::getTicks(uint8_t motorIdx) const {
  if (motorIdx >= 4) return 0;
  noInterrupts();
  const int32_t ticks = encoderTicks[motorIdx];
  interrupts();
  return ticks;
}

void EncoderManager::resetTicks() {
  noInterrupts();
  for (uint8_t i = 0; i < 4; ++i) {
    encoderTicks[i] = 0;
    sampleTicks_[i] = 0;
    rpm_[i] = 0.0f;
  }
  lastSampleMs_ = millis();
  interrupts();
}

void EncoderManager::reset() {
  resetTicks();
}

float EncoderManager::getRPM(uint8_t motor) const {
  if (motor >= 4) return 0.0f;
  return rpm_[motor];
}

float EncoderManager::getDistanceMm(uint8_t motor) const {
  if (motor >= 4) return 0.0f;
  const float wheelCircumferenceMm = PI * WHEEL_DIAMETER_MM;
  const float ticksPerRev = ticksPerOutputRev(motor);
  if (ticksPerRev <= 0.0f) return 0.0f;
  return (static_cast<float>(getTicks(motor)) / ticksPerRev) * wheelCircumferenceMm;
}

float EncoderManager::getVelocityMmPerSec(uint8_t motor) const {
  if (motor >= 4) return 0.0f;
  const float wheelCircumferenceMm = PI * WHEEL_DIAMETER_MM;
  const float rpm = getRPM(motor);
  return (rpm / 60.0f) * wheelCircumferenceMm;
}
