#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "../src/config.h"

// Pines de Motores (M1..M4)
// M1: Front-Left
constexpr uint8_t M1_PWM = MOTOR1_PWM_PIN;
constexpr uint8_t M1_IN1 = MOTOR1_AIN1;
constexpr uint8_t M1_IN2 = MOTOR1_AIN2;

// M2: Front-Right
constexpr uint8_t M2_PWM = MOTOR2_PWM_PIN;
constexpr uint8_t M2_IN1 = MOTOR2_BIN1;
constexpr uint8_t M2_IN2 = MOTOR2_BIN2;

// M3: Back-Left
constexpr uint8_t M3_PWM = MOTOR3_PWM_PIN;
constexpr uint8_t M3_IN1 = MOTOR3_AIN1;
constexpr uint8_t M3_IN2 = MOTOR3_AIN2;

// M4: Back-Right
constexpr uint8_t M4_PWM = MOTOR4_PWM_PIN;
constexpr uint8_t M4_IN1 = MOTOR4_BIN1;
constexpr uint8_t M4_IN2 = MOTOR4_BIN2;

// Variables atómicas de conteo de encoders
volatile int32_t encTicks[4] = {0, 0, 0, 0};
static volatile uint8_t prevPortB = 0;
static volatile uint8_t prevPortJ = 0;
static volatile uint8_t prevState[4] = {0, 0, 0, 0};

// Tabla de transiciones de cuadratura
static const int8_t kQuadTable[16] = {
  0,  1, -1,  0,
 -1,  0,  0,  1,
  1,  0,  0, -1,
  0, -1,  1,  0
};

// Estados actuales de PWM asignados a cada motor
int16_t currentMotorPWM[4] = {0, 0, 0, 0};

// Rutina de interrupción externa hardware para Encoder 4
void isrEncoder4() {
  if (digitalRead(ENC4_PIN_B) == HIGH) {
    encTicks[3] += ENCODER4_INVERTED ? -1 : 1;
  } else {
    encTicks[3] += ENCODER4_INVERTED ? 1 : -1;
  }
}

// Interrupción Pin Change 0 para M1 (D10/D11: PB4/PB5) y M2 (D12/D13: PB6/PB7)
ISR(PCINT0_vect) {
  uint8_t currB = PINB;
  uint8_t diffB = currB ^ prevPortB;

  if (diffB & (_BV(PB4) | _BV(PB5))) {
    uint8_t curr = ((currB >> PB4) & 1) | (((currB >> PB5) & 1) << 1);
    uint8_t idx = (prevState[0] << 2) | curr;
    int8_t diff = kQuadTable[idx];
    if (diff != 0) {
      encTicks[0] += ENCODER1_INVERTED ? -diff : diff;
      prevState[0] = curr;
    }
  }

  if (diffB & (_BV(PB6) | _BV(PB7))) {
    uint8_t curr = ((currB >> PB6) & 1) | (((currB >> PB7) & 1) << 1);
    uint8_t idx = (prevState[1] << 2) | curr;
    int8_t diff = kQuadTable[idx];
    if (diff != 0) {
      encTicks[1] += ENCODER2_INVERTED ? -diff : diff;
      prevState[1] = curr;
    }
  }

  prevPortB = currB;
}

// Interrupción Pin Change 1 para M3 (D14/D15: PJ1/PJ0)
ISR(PCINT1_vect) {
  uint8_t currJ = PINJ;
  uint8_t diffJ = currJ ^ prevPortJ;

  if (diffJ & (_BV(PJ0) | _BV(PJ1))) {
    uint8_t curr = ((currJ >> PJ1) & 1) | (((currJ >> PJ0) & 1) << 1);
    uint8_t idx = (prevState[2] << 2) | curr;
    int8_t diff = kQuadTable[idx];
    if (diff != 0) {
      encTicks[2] += ENCODER3_INVERTED ? -diff : diff;
      prevState[2] = curr;
    }
  }

  prevPortJ = currJ;
}

// Control individual de motores con soporte de inversión lógica
void setRawMotor(uint8_t motor, int16_t pwm) {
  if (motor >= 4) return;
  currentMotorPWM[motor] = pwm;

  bool inv = false;
  uint8_t pwmPin = 0, in1Pin = 0, in2Pin = 0;
  switch (motor) {
    case 0: inv = MOTOR1_INVERTED; pwmPin = M1_PWM; in1Pin = M1_IN1; in2Pin = M1_IN2; break;
    case 1: inv = MOTOR2_INVERTED; pwmPin = M2_PWM; in1Pin = M2_IN1; in2Pin = M2_IN2; break;
    case 2: inv = MOTOR3_INVERTED; pwmPin = M3_PWM; in1Pin = M3_IN1; in2Pin = M3_IN2; break;
    case 3: inv = MOTOR4_INVERTED; pwmPin = M4_PWM; in1Pin = M4_IN1; in2Pin = M4_IN2; break;
  }

  int16_t effPwm = pwm;
  if (inv) effPwm = -effPwm;

  if (effPwm > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, effPwm > 255 ? 255 : effPwm);
  } else if (effPwm < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    int16_t mag = -effPwm;
    analogWrite(pwmPin, mag > 255 ? 255 : mag);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void setAllMotors(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
  setRawMotor(0, m1);
  setRawMotor(1, m2);
  setRawMotor(2, m3);
  setRawMotor(3, m4);
}

void stopAllMotors() {
  setAllMotors(0, 0, 0, 0);
}

// Helper para leer ticks de manera atómica
void readEncoderTicks(int32_t outTicks[4]) {
  noInterrupts();
  outTicks[0] = encTicks[0];
  outTicks[1] = encTicks[1];
  outTicks[2] = encTicks[2];
  outTicks[3] = encTicks[3];
  interrupts();
}

// Telemetría periódica
void printTelemetry() {
  static uint32_t lastTeleMs = 0;
  if (millis() - lastTeleMs >= 100UL) {
    lastTeleMs = millis();
    int32_t t[4];
    readEncoderTicks(t);

    Serial.print(F("PWM: [M1:"));
    Serial.print(currentMotorPWM[0]);
    Serial.print(F(" M2:"));
    Serial.print(currentMotorPWM[1]);
    Serial.print(F(" M3:"));
    Serial.print(currentMotorPWM[2]);
    Serial.print(F(" M4:"));
    Serial.print(currentMotorPWM[3]);
    Serial.print(F("] | Enc1: "));
    Serial.print(t[0]);
    Serial.print(F(" | Enc2: "));
    Serial.print(t[1]);
    Serial.print(F(" | Enc3: "));
    Serial.print(t[2]);
    Serial.print(F(" | Enc4: "));
    Serial.println(t[3]);
  }
}

void printMenu() {
  Serial.println(F("\n=============================================="));
  Serial.println(F(" TEST 08: MOTORES MECANUM Y ENCODERS (TESTS 2.0)"));
  Serial.println(F("=============================================="));
  Serial.println(F("Comandos disponibles (escribe en Serial):"));
  Serial.println(F("  'r' : Rampa progresiva PWM 0-255 (Zona muerta/Deadband)"));
  Serial.println(F("  'd' : Prueba de direccion (Avanzar, Retroceder, Giro)"));
  Serial.println(F("  's' : Detener todos los motores"));
  Serial.println(F("  'z' : Poner contadores de encoders a cero"));
  Serial.println(F("  'm' : Mostrar este menu"));
  Serial.println(F("==============================================\n"));
}

// a) Módulo de rampa de velocidad
void runDeadbandRamp() {
  Serial.println(F("\n--- INICIANDO RAMPA DE VELOCIDAD PWM 0 -> 255 ---"));
  Serial.println(F("Observa el PWM cuando los ticks comiencen a cambiar para cada motor."));

  int32_t startTicks[4];
  readEncoderTicks(startTicks);

  int16_t deadband[4] = {-1, -1, -1, -1};

  for (int16_t pwm = 0; pwm <= 255; pwm += 5) {
    setAllMotors(pwm, pwm, pwm, pwm);

    uint32_t stepStart = millis();
    while (millis() - stepStart < 120UL) {
      printTelemetry();
      yield();
    }

    int32_t currTicks[4];
    readEncoderTicks(currTicks);

    for (uint8_t i = 0; i < 4; ++i) {
      if (deadband[i] < 0 && abs(currTicks[i] - startTicks[i]) >= 5) {
        deadband[i] = pwm;
        Serial.print(F(">>> Motor M"));
        Serial.print(i + 1);
        Serial.print(F(" comenzo a girar a PWM: "));
        Serial.println(pwm);
      }
    }
  }

  stopAllMotors();
  Serial.println(F("\n--- RESULTADOS ZONA MUERTA (DEADBAND) ---"));
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(F("Motor M"));
    Serial.print(i + 1);
    Serial.print(F(": PWM min = "));
    if (deadband[i] >= 0) Serial.println(deadband[i]);
    else Serial.println(F("No detectado"));
  }
  Serial.println(F("----------------------------------------\n"));
}

// b) Prueba de dirección: 1s avanzar, 1s retroceder, 1s girar
void runDirectionTest() {
  Serial.println(F("\n--- INICIANDO PRUEBA DE DIRECCION (PWM 120) ---"));

  // 1. Avanzar 1 segundo
  Serial.println(F("1/3: Avanzar 1 segundo..."));
  setAllMotors(120, 120, 120, 120);
  uint32_t start = millis();
  while (millis() - start < 1000UL) {
    printTelemetry();
    yield();
  }

  // Pausa intermedia
  stopAllMotors();
  start = millis();
  while (millis() - start < 400UL) {
    printTelemetry();
    yield();
  }

  // 2. Retroceder 1 segundo
  Serial.println(F("2/3: Retroceder 1 segundo..."));
  setAllMotors(-120, -120, -120, -120);
  start = millis();
  while (millis() - start < 1000UL) {
    printTelemetry();
    yield();
  }

  // Pausa intermedia
  stopAllMotors();
  start = millis();
  while (millis() - start < 400UL) {
    printTelemetry();
    yield();
  }

  // 3. Girar sobre su propio eje (spin) 1 segundo
  // Giro horario (CW): lado izquierdo adelante, lado derecho atras
  Serial.println(F("3/3: Girar sobre propio eje (spin) 1 segundo..."));
  setAllMotors(120, -120, 120, -120);
  start = millis();
  while (millis() - start < 1000UL) {
    printTelemetry();
    yield();
  }

  stopAllMotors();
  Serial.println(F("--- PRUEBA DE DIRECCION FINALIZADA ---\n"));
}

void setup() {
  Serial.begin(115200);

  // Configuración de pines de motores
  const uint8_t motorOutputs[] = {
    M1_PWM, M1_IN1, M1_IN2,
    M2_PWM, M2_IN1, M2_IN2,
    M3_PWM, M3_IN1, M3_IN2,
    M4_PWM, M4_IN1, M4_IN2
  };
  for (uint8_t pin : motorOutputs) {
    pinMode(pin, OUTPUT);
  }
  stopAllMotors();

  // Configuración de pines de encoders con pull-up
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);
  pinMode(ENCODER3_A, INPUT_PULLUP);
  pinMode(ENCODER3_B, INPUT_PULLUP);
  pinMode(ENCODER4_A, INPUT_PULLUP);
  pinMode(ENCODER4_B, INPUT_PULLUP);

  noInterrupts();
  prevPortB = PINB;
  prevPortJ = PINJ;
  prevState[0] = ((PINB >> PB4) & 1) | (((PINB >> PB5) & 1) << 1);
  prevState[1] = ((PINB >> PB6) & 1) | (((PINB >> PB7) & 1) << 1);
  prevState[2] = ((PINJ >> PJ1) & 1) | (((PINJ >> PJ0) & 1) << 1);

  // Habilitar Pin Change Interrupts para D10-D13 (PCINT0) y D14-D15 (PCINT1)
  PCICR |= _BV(PCIE0) | _BV(PCIE1);
  PCMSK0 |= _BV(PCINT4) | _BV(PCINT5) | _BV(PCINT6) | _BV(PCINT7);
  PCMSK1 |= _BV(PCINT9) | _BV(PCINT10);
  interrupts();

  int intNum4 = digitalPinToInterrupt(ENC4_PIN_A);
  if (intNum4 == NOT_AN_INTERRUPT) {
    Serial.println(F("[ERROR CRÍTICO] ENC4_PIN_A no soporta attachInterrupt()"));
  } else {
    attachInterrupt(intNum4, isrEncoder4, RISING);
  }

  Serial.print("Encoder 1 Int Pin: "); Serial.println(digitalPinToInterrupt(ENC1_PIN_A));
  Serial.print("Encoder 2 Int Pin: "); Serial.println(digitalPinToInterrupt(ENC2_PIN_A));
  Serial.print("Encoder 3 Int Pin: "); Serial.println(digitalPinToInterrupt(ENC3_PIN_A));
  Serial.print("Encoder 4 Int Pin: "); Serial.println(digitalPinToInterrupt(ENC4_PIN_A));

  printMenu();
}

void loop() {
  printTelemetry();

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    while (Serial.available() > 0) Serial.read(); // Descartar resto del buffer

    switch (cmd) {
      case 'r':
      case 'R':
        runDeadbandRamp();
        break;
      case 'd':
      case 'D':
        runDirectionTest();
        break;
      case 's':
      case 'S':
        stopAllMotors();
        Serial.println(F("[MOTORES DETENIDOS]"));
        break;
      case 'z':
      case 'Z':
        noInterrupts();
        encTicks[0] = encTicks[1] = encTicks[2] = encTicks[3] = 0;
        interrupts();
        Serial.println(F("[ENCODERS REINICIADOS A CERO]"));
        break;
      case 'm':
      case 'M':
        printMenu();
        break;
      default:
        break;
    }
  }

  yield();
}
