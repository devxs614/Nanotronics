#include "EncoderManager.h"
#include <avr/interrupt.h>
#include <avr/io.h>

volatile int32_t encoderTicks[4] = {0, 0, 0, 0};
EncoderManager* EncoderManager::instance_ = nullptr;
volatile uint8_t EncoderManager::previousPortB_ = 0;
volatile uint8_t EncoderManager::previousPortJ_ = 0;
volatile uint8_t EncoderManager::previousPortH_ = 0;

namespace {
const uint8_t kQuadratureTransitions[16] = {
  0, 1, 255, 0,
  255, 0, 0, 1,
  1, 0, 0, 255,
  0, 255, 1, 0
};
}

void EncoderManager::begin() {
  instance_ = this;

  for (uint8_t i = 0; i < 4; ++i) {
    previousTicks[i] = 0;
    previousMicros[i] = micros();
  }
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
  previousPortH_ = PINH;

  // D10-D13 are PB4-PB7 and D14-D15 are PJ1-PJ0 on the Mega 2560.
  PCICR |= _BV(PCIE0) | _BV(PCIE1);
  PCMSK0 |= _BV(PCINT4) | _BV(PCINT5) | _BV(PCINT6) | _BV(PCINT7);
  PCMSK1 |= _BV(PCINT16) | _BV(PCINT17);

  // D16-D17 are PH1-PH0 and have no PCINT capability on ATmega2560.
  // Timer2 samples this pair at 20 kHz without using attachInterrupt().
  // The Servo library reserves the 16-bit timers on the Mega.
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS21);
  OCR2A = 99;
  TIMSK2 |= _BV(OCIE2A);
  interrupts();
}

void EncoderManager::update() {
  for (uint8_t motor = 0; motor < 4; ++motor) {
    previousTicks[motor] = encoderTicks[motor];
    previousMicros[motor] = micros();
  }
}

void EncoderManager::processQuadrature(uint8_t motor, uint8_t state) {
  static volatile uint8_t previousState[4] = {0, 0, 0, 0};
  const uint8_t transition = (previousState[motor] << 2) | (state & 0x03);
  const uint8_t direction = kQuadratureTransitions[transition];
  previousState[motor] = state & 0x03;

  if (direction == 1) {
    ++encoderTicks[motor];
  } else if (direction == 255) {
    --encoderTicks[motor];
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

void EncoderManager::processPortH(uint8_t portState) {
  const uint8_t changed = portState ^ previousPortH_;
  previousPortH_ = portState;

  if (changed & (_BV(PH1) | _BV(PH0))) {
    processQuadrature(3, ((portState >> PH1) & 1) | (((portState >> PH0) & 1) << 1));
  }
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
  static const float ticksPerRev = 260.0f;
  const uint32_t elapsedMicros = micros() - previousMicros[motor];
  if (elapsedMicros == 0) return 0.0f;
  const float deltaTicks = static_cast<float>(encoderTicks[motor] - previousTicks[motor]);
  const float revPerSecond = (deltaTicks / ticksPerRev) / (elapsedMicros / 1000000.0f);
  return revPerSecond * 60.0f;
}

float EncoderManager::getDistanceMm(uint8_t motor) const {
  if (motor >= 4) return 0.0f;
  const float wheelCircumferenceMm = PI * WHEEL_DIAMETER_MM;
  return (static_cast<float>(encoderTicks[motor]) / ENCODER_TICKS_PER_OUTPUT_REV_M1) * wheelCircumferenceMm;
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
    previousTicks[i] = 0;
  }
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

ISR(TIMER2_COMPA_vect) {
  if (EncoderManager::instance_ != nullptr) {
    EncoderManager::processPortH(PINH);
  }
}
