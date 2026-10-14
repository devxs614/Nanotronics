#include "EncoderManager.h"

// Contadores de ticks absolutos (modificados desde las ISR)
volatile int32_t enc1_ticks = 0;
volatile int32_t enc2_ticks = 0;
volatile int32_t enc3_ticks = 0;
volatile int32_t enc4_ticks = 0;

// Flanco de subida en el canal A; el nivel del canal B define el sentido.
void isrEncoder1() {
  if ((digitalRead(ENC1_PIN_B) == HIGH) != ENCODER1_INVERTED) enc1_ticks++; else enc1_ticks--;
}
void isrEncoder2() {
  if ((digitalRead(ENC2_PIN_B) == HIGH) != ENCODER2_INVERTED) enc2_ticks++; else enc2_ticks--;
}
void isrEncoder3() {
  if ((digitalRead(ENC3_PIN_B) == HIGH) != ENCODER3_INVERTED) enc3_ticks++; else enc3_ticks--;
}
void isrEncoder4() {
  if ((digitalRead(ENC4_PIN_B) == HIGH) != ENCODER4_INVERTED) enc4_ticks++; else enc4_ticks--;
}

namespace {
float ticksPerOutputRev(uint8_t motor) {
  switch (motor) {
    case 0: return ENCODER_TICKS_PER_OUTPUT_REV_M1;
    case 1: return ENCODER_TICKS_PER_OUTPUT_REV_M2;
    case 2: return ENCODER_TICKS_PER_OUTPUT_REV_M3;
    default: return ENCODER_TICKS_PER_OUTPUT_REV_M4;
  }
}

// Lectura atomica del contador del motor indicado (0..3)
int32_t readTicksAtomic(uint8_t motor) {
  noInterrupts();
  int32_t tempTicks;
  switch (motor) {
    case 0:  tempTicks = enc1_ticks; break;
    case 1:  tempTicks = enc2_ticks; break;
    case 2:  tempTicks = enc3_ticks; break;
    default: tempTicks = enc4_ticks; break;
  }
  interrupts();
  return tempTicks;
}

void attachEncoderInterrupt(uint8_t index, uint8_t pinA, void (*isrFunc)()) {
  const int intNum = digitalPinToInterrupt(pinA);
  if (intNum != NOT_AN_INTERRUPT) {
    attachInterrupt(intNum, isrFunc, RISING);
  } else {
    Serial.print(F("[ERROR ENCODER] Pin sin interrupcion (Enc"));
    Serial.print(index);
    Serial.print(F("): "));
    Serial.println(pinA);
  }
}
}

void EncoderManager::begin() {
  for (uint8_t i = 0; i < 4; ++i) {
    sampleTicks_[i] = 0;
    rpm_[i] = 0.0f;
  }
  noInterrupts();
  enc1_ticks = enc2_ticks = enc3_ticks = enc4_ticks = 0;
  interrupts();
  lastSampleMs_ = millis();

  pinMode(ENC1_PIN_A, INPUT_PULLUP); pinMode(ENC1_PIN_B, INPUT_PULLUP);
  pinMode(ENC2_PIN_A, INPUT_PULLUP); pinMode(ENC2_PIN_B, INPUT_PULLUP);
  pinMode(ENC3_PIN_A, INPUT_PULLUP); pinMode(ENC3_PIN_B, INPUT_PULLUP);
  pinMode(ENC4_PIN_A, INPUT_PULLUP); pinMode(ENC4_PIN_B, INPUT_PULLUP);

  attachEncoderInterrupt(1, ENC1_PIN_A, isrEncoder1);
  attachEncoderInterrupt(2, ENC2_PIN_A, isrEncoder2);
  attachEncoderInterrupt(3, ENC3_PIN_A, isrEncoder3);
  attachEncoderInterrupt(4, ENC4_PIN_A, isrEncoder4);

  Serial.print(F("[ENCODER SETUP] Enc1(Pin ")); Serial.print(ENC1_PIN_A); Serial.print(F(")->Int:")); Serial.println(digitalPinToInterrupt(ENC1_PIN_A));
  Serial.print(F("[ENCODER SETUP] Enc2(Pin ")); Serial.print(ENC2_PIN_A); Serial.print(F(")->Int:")); Serial.println(digitalPinToInterrupt(ENC2_PIN_A));
  Serial.print(F("[ENCODER SETUP] Enc3(Pin ")); Serial.print(ENC3_PIN_A); Serial.print(F(")->Int:")); Serial.println(digitalPinToInterrupt(ENC3_PIN_A));
  Serial.print(F("[ENCODER SETUP] Enc4(Pin ")); Serial.print(ENC4_PIN_A); Serial.print(F(")->Int:")); Serial.println(digitalPinToInterrupt(ENC4_PIN_A));
}

void EncoderManager::update() {
  const uint32_t now = millis();
  const uint32_t elapsedMs = now - lastSampleMs_;
  if (elapsedMs < ENCODER_RPM_SAMPLE_MS) return;

  int32_t currentTicks[4];
  for (uint8_t motor = 0; motor < 4; ++motor) {
    currentTicks[motor] = readTicksAtomic(motor);
  }

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
  return readTicksAtomic(motorIdx);
}

void EncoderManager::resetTicks() {
  noInterrupts();
  enc1_ticks = enc2_ticks = enc3_ticks = enc4_ticks = 0;
  for (uint8_t i = 0; i < 4; ++i) {
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
