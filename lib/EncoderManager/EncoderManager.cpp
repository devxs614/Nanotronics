#include "EncoderManager.h"
#include <avr/interrupt.h>
#include <avr/io.h>

volatile int32_t encoderTicks[4] = {0, 0, 0, 0};
EncoderManager* EncoderManager::instance_ = nullptr;
volatile uint8_t EncoderManager::previousPortB_ = 0;
volatile uint8_t EncoderManager::previousPortJ_ = 0;
volatile uint8_t EncoderManager::previousState_[4] = {0, 0, 0, 0};

namespace {
const uint8_t kQuadratureTransitions[16] = {
  0, 1, 255, 0,
  255, 0, 0, 1,
  1, 0, 0, 255,
  0, 255, 1, 0
};

float ticksPerOutputRev(uint8_t motor) {
  switch (motor) {
    case 0: return ENCODER_TICKS_PER_OUTPUT_REV_M1;
    case 1: return ENCODER_TICKS_PER_OUTPUT_REV_M2;
    case 2: return ENCODER_TICKS_PER_OUTPUT_REV_M3;
    default: return ENCODER_TICKS_PER_OUTPUT_REV_M4;
  }
}

bool encoderInverted(uint8_t motor) {
  switch (motor) {
    case 0: return ENCODER1_INVERTED;
    case 1: return ENCODER2_INVERTED;
    case 2: return ENCODER3_INVERTED;
    default: return ENCODER4_INVERTED;
  }
}

void motor4EncoderAChanged() {
  EncoderManager::handleMotor4Interrupt();
}

void motor4EncoderBChanged() {
  EncoderManager::handleMotor4Interrupt();
}
}

void EncoderManager::begin() {
  instance_ = this;

  for (uint8_t i = 0; i < 4; ++i) {
    sampleTicks_[i] = 0;
    rpm_[i] = 0.0f;
  }
  lastSampleMs_ = millis();
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);
  pinMode(ENCODER3_A, INPUT_PULLUP);
  pinMode(ENCODER3_B, INPUT_PULLUP);
  pinMode(ENCODER4_A, INPUT_PULLUP);
  pinMode(ENCODER4_B, INPUT_PULLUP);

  noInterrupts();
  previousPortB_ = PINB;
  previousPortJ_ = PINJ;
  previousState_[0] = ((PINB >> PB4) & 1) | (((PINB >> PB5) & 1) << 1);
  previousState_[1] = ((PINB >> PB6) & 1) | (((PINB >> PB7) & 1) << 1);
  previousState_[2] = ((PINJ >> PJ1) & 1) | (((PINJ >> PJ0) & 1) << 1);

  // D10-D13 are PB4-PB7 and D14-D15 are PJ1-PJ0 on the Mega 2560.
  PCICR |= _BV(PCIE0) | _BV(PCIE1);
  PCMSK0 |= _BV(PCINT4) | _BV(PCINT5) | _BV(PCINT6) | _BV(PCINT7);
  PCMSK1 |= _BV(PCINT9) | _BV(PCINT10);
  interrupts();

  // D18 and D19 map to native external interrupts INT3 and INT2 on the Mega.
  previousState_[3] = ((PIND >> PD3) & 1) | (((PIND >> PD2) & 1) << 1);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_A), motor4EncoderAChanged, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_B), motor4EncoderBChanged, CHANGE);
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

void EncoderManager::processQuadrature(uint8_t motor, uint8_t state) {
  const uint8_t transition = (previousState_[motor] << 2) | (state & 0x03);
  const uint8_t direction = kQuadratureTransitions[transition];
  previousState_[motor] = state & 0x03;

  if (direction == 1) {
    encoderTicks[motor] += encoderInverted(motor) ? -1 : 1;
  } else if (direction == 255) {
    encoderTicks[motor] += encoderInverted(motor) ? 1 : -1;
  }
}

void EncoderManager::processPortB(uint8_t portState) {
  const uint8_t changed = portState ^ previousPortB_;
  previousPortB_ = portState;

  if (changed & (_BV(PB4) | _BV(PB5))) {
    processQuadrature(0, ((portState >> PB4) & 1) | (((portState >> PB5) & 1) << 1));
  }
  if (changed & (_BV(PB6) | _BV(PB7))) {
    processQuadrature(1, ((portState >> PB6) & 1) | (((portState >> PB7) & 1) << 1));
  }
}

void EncoderManager::processPortJ(uint8_t portState) {
  const uint8_t changed = portState ^ previousPortJ_;
  previousPortJ_ = portState;

  if (changed & (_BV(PJ1) | _BV(PJ0))) {
    processQuadrature(2, ((portState >> PJ1) & 1) | (((portState >> PJ0) & 1) << 1));
  }
}

void EncoderManager::handleMotor4Interrupt() {
  const uint8_t portState = PIND;
  processQuadrature(3, ((portState >> PD3) & 1) | (((portState >> PD2) & 1) << 1));
}

int32_t EncoderManager::getTicks(uint8_t motor) const {
  if (motor >= 4) return 0;
  noInterrupts();
  const int32_t ticks = encoderTicks[motor];
  interrupts();
  return ticks;
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

void EncoderManager::reset() {
  noInterrupts();
  for (uint8_t i = 0; i < 4; ++i) {
    encoderTicks[i] = 0;
    sampleTicks_[i] = 0;
    rpm_[i] = 0.0f;
  }
  lastSampleMs_ = millis();
  interrupts();
}

ISR(PCINT0_vect) {
  if (EncoderManager::instance_ != nullptr) {
    EncoderManager::processPortB(PINB);
  }
}

ISR(PCINT1_vect) {
  if (EncoderManager::instance_ != nullptr) {
    EncoderManager::processPortJ(PINJ);
  }
}
