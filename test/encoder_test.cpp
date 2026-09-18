#include <Arduino.h>
#include <avr/interrupt.h>

volatile int32_t encTicksM1 = 0;
volatile int32_t encTicksM2 = 0;
volatile int32_t encTicksM3 = 0;
volatile int32_t encTicksM4 = 0;

static uint8_t lastPortB = 0;
static uint8_t lastPortJ = 0;

// ISR para M4 usando Interrupción Externa Hardware (Pin D18)
void doEncoderM4() {
  bool pinA = digitalRead(18);
  bool pinB = digitalRead(19);
  encTicksM4 += (pinA == pinB) ? 1 : -1;
}

void setup() {
  Serial.begin(115200);

  // M1 (D10/D11) y M2 (D12/D13) en PORTB
  DDRB &= ~((1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));
  PORTB |= ((1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));

  // M3 (D14/D15) en PORTJ
  DDRJ &= ~((1 << PJ0) | (1 << PJ1));
  PORTJ |= ((1 << PJ0) | (1 << PJ1));

  // M4 en D18 (INT3) y D19
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(18), doEncoderM4, CHANGE);

  cli();
  PCICR |= (1 << PCIE0) | (1 << PCIE1);
  PCMSK0 |= (1 << PCINT4) | (1 << PCINT5) | (1 << PCINT6) | (1 << PCINT7); // D10-D13
  PCMSK1 |= (1 << PCINT9) | (1 << PCINT10); // D14-D15

  lastPortB = PINB;
  lastPortJ = PINJ;
  sei();

  Serial.println("=== TEST DE TICKS (4 MOTORES CON INTERRUPCIONES NATIVAS) ===");
}

ISR(PCINT0_vect) {
  uint8_t currentB = PINB;
  uint8_t diffB = currentB ^ lastPortB;

  if (diffB & (1 << PB4)) { // M1
    bool pinA = currentB & (1 << PB4);
    bool pinB = currentB & (1 << PB5);
    encTicksM1 += (pinA == pinB) ? 1 : -1;
  }
  if (diffB & (1 << PB6)) { // M2
    bool pinA = currentB & (1 << PB6);
    bool pinB = currentB & (1 << PB7);
    encTicksM2 += (pinA == pinB) ? 1 : -1;
  }
  lastPortB = currentB;
}

ISR(PCINT1_vect) {
  uint8_t currentJ = PINJ;
  uint8_t diffJ = currentJ ^ lastPortJ;

  if (diffJ & (1 << PJ1)) { // M3
    bool pinA = currentJ & (1 << PJ1);
    bool pinB = currentJ & (1 << PJ0);
    encTicksM3 += (pinA == pinB) ? 1 : -1;
  }
  lastPortJ = currentJ;
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 150) {
    lastPrint = millis();

    cli();
    int32_t t1 = encTicksM1;
    int32_t t2 = encTicksM2;
    int32_t t3 = encTicksM3;
    int32_t t4 = encTicksM4;
    sei();

    Serial.print("M1 (FR): "); Serial.print(t1);
    Serial.print(" | M2 (FL): "); Serial.print(t2);
    Serial.print(" | M3 (RR): "); Serial.print(t3);
    Serial.print(" | M4 (RL): "); Serial.println(t4);
  }
}