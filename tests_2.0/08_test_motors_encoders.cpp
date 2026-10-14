#include <Arduino.h>
#include "../src/config.h"

// Pines de Motores (M1..M4)
constexpr uint8_t M1_PWM = MOTOR1_PWM_PIN;
constexpr uint8_t M1_IN1 = MOTOR1_AIN1;
constexpr uint8_t M1_IN2 = MOTOR1_AIN2;

constexpr uint8_t M2_PWM = MOTOR2_PWM_PIN;
constexpr uint8_t M2_IN1 = MOTOR2_BIN1;
constexpr uint8_t M2_IN2 = MOTOR2_BIN2;

constexpr uint8_t M3_PWM = MOTOR3_PWM_PIN;
constexpr uint8_t M3_IN1 = MOTOR3_AIN1;
constexpr uint8_t M3_IN2 = MOTOR3_AIN2;

constexpr uint8_t M4_PWM = MOTOR4_PWM_PIN;
constexpr uint8_t M4_IN1 = MOTOR4_BIN1;
constexpr uint8_t M4_IN2 = MOTOR4_BIN2;

constexpr int32_t DEADBAND_TICKS = 5;

// Conteo atómico de encoders
volatile int32_t encTicks[4] = {0, 0, 0, 0};
int16_t currentMotorPWM[4] = {0, 0, 0, 0};

// Rutinas de Interrupción Hardware (Canal A -> RISING, lectura de Canal B)
static inline void encoderPulse(uint8_t idx, uint8_t pinB, bool inverted) {
  const bool forward = (digitalRead(pinB) == HIGH);
  encTicks[idx] += (forward != inverted) ? 1 : -1;
}

void isrEncoder1() { encoderPulse(0, ENC1_PIN_B, ENCODER1_INVERTED); }
void isrEncoder2() { encoderPulse(1, ENC2_PIN_B, ENCODER2_INVERTED); }
void isrEncoder3() { encoderPulse(2, ENC3_PIN_B, ENCODER3_INVERTED); }
void isrEncoder4() { encoderPulse(3, ENC4_PIN_B, ENCODER4_INVERTED); }

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

  int16_t effPwm = inv ? -pwm : pwm;

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
  setRawMotor(0, m1); setRawMotor(1, m2); setRawMotor(2, m3); setRawMotor(3, m4);
}

void stopAllMotors() { setAllMotors(0, 0, 0, 0); }

void readEncoderTicks(int32_t outTicks[4]) {
  noInterrupts();
  outTicks[0] = encTicks[0]; outTicks[1] = encTicks[1];
  outTicks[2] = encTicks[2]; outTicks[3] = encTicks[3];
  interrupts();
}

void printTelemetry() {
  static uint32_t lastTeleMs = 0;
  if (millis() - lastTeleMs >= 100UL) {
    lastTeleMs = millis();
    int32_t t[4];
    readEncoderTicks(t);

    Serial.print(F("PWM: [M1:")); Serial.print(currentMotorPWM[0]);
    Serial.print(F(" M2:")); Serial.print(currentMotorPWM[1]);
    Serial.print(F(" M3:")); Serial.print(currentMotorPWM[2]);
    Serial.print(F(" M4:")); Serial.print(currentMotorPWM[3]);
    Serial.print(F("] | Enc1: ")); Serial.print(t[0]);
    Serial.print(F(" | Enc2: ")); Serial.print(t[1]);
    Serial.print(F(" | Enc3: ")); Serial.print(t[2]);
    Serial.print(F(" | Enc4: ")); Serial.println(t[3]);
  }
}

void printMenu() {
  Serial.println(F("\n=============================================="));
  Serial.println(F(" TEST 08: MOTORES MECANUM Y ENCODERS (TESTS 2.0)"));
  Serial.println(F("=============================================="));
  Serial.println(F("Comandos disponibles:"));
  Serial.println(F("  'r' : Rampa progresiva PWM 0-255 (Zona muerta)"));
  Serial.println(F("  'd' : Prueba de direccion (Avanzar, Retroceder, Giro)"));
  Serial.println(F("  's' : Detener todos los motores"));
  Serial.println(F("  'z' : Poner contadores de encoders a cero"));
  Serial.println(F("  'm' : Mostrar este menu"));
  Serial.println(F("==============================================\n"));
}

void runDeadbandRamp() {
  Serial.println(F("\n--- INICIANDO RAMPA DE VELOCIDAD PWM 0 -> 255 ---"));
  int32_t startTicks[4]; readEncoderTicks(startTicks);
  int16_t deadband[4] = {-1, -1, -1, -1};

  for (int16_t pwm = 0; pwm <= 255; pwm += 5) {
    setAllMotors(pwm, pwm, pwm, pwm);
    uint32_t stepStart = millis();
    while (millis() - stepStart < 120UL) { printTelemetry(); yield(); }

    int32_t currTicks[4]; readEncoderTicks(currTicks);
    for (uint8_t i = 0; i < 4; ++i) {
      if (deadband[i] < 0 && abs(currTicks[i] - startTicks[i]) >= DEADBAND_TICKS) {
        deadband[i] = pwm;
        Serial.print(F(">>> Motor M")); Serial.print(i + 1);
        Serial.print(F(" comenzo a girar a PWM: ")); Serial.println(pwm);
      }
    }
  }

  stopAllMotors();
  Serial.println(F("\n--- RESULTADOS ZONA MUERTA (DEADBAND) ---"));
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(F("Motor M")); Serial.print(i + 1); Serial.print(F(": PWM min = "));
    if (deadband[i] >= 0) Serial.println(deadband[i]); else Serial.println(F("No detectado"));
  }
}

void runDirectionTest() {
  Serial.println(F("\n--- INICIANDO PRUEBA DE DIRECCION (PWM 120) ---"));
  Serial.println(F("1/3: Avanzar 1 segundo..."));
  setAllMotors(120, 120, 120, 120);
  uint32_t start = millis();
  while (millis() - start < 1000UL) { printTelemetry(); yield(); }

  stopAllMotors(); start = millis();
  while (millis() - start < 400UL) { printTelemetry(); yield(); }

  Serial.println(F("2/3: Retroceder 1 segundo..."));
  setAllMotors(-120, -120, -120, -120);
  start = millis();
  while (millis() - start < 1000UL) { printTelemetry(); yield(); }

  stopAllMotors(); start = millis();
  while (millis() - start < 400UL) { printTelemetry(); yield(); }

  Serial.println(F("3/3: Girar sobre propio eje (spin) 1 segundo..."));
  setAllMotors(120, -120, 120, -120);
  start = millis();
  while (millis() - start < 1000UL) { printTelemetry(); yield(); }

  stopAllMotors();
  Serial.println(F("--- PRUEBA DE DIRECCION FINALIZADA ---\n"));
}

void setup() {
  Serial.begin(115200);

  const uint8_t motorOutputs[] = {
    M1_PWM, M1_IN1, M1_IN2, M2_PWM, M2_IN1, M2_IN2,
    M3_PWM, M3_IN1, M3_IN2, M4_PWM, M4_IN1, M4_IN2
  };
  for (uint8_t pin : motorOutputs) pinMode(pin, OUTPUT);
  stopAllMotors();

  const uint8_t encPinsA[4] = {ENC1_PIN_A, ENC2_PIN_A, ENC3_PIN_A, ENC4_PIN_A};
  const uint8_t encPinsB[4] = {ENC1_PIN_B, ENC2_PIN_B, ENC3_PIN_B, ENC4_PIN_B};
  void (*const encIsrs[4])() = {isrEncoder1, isrEncoder2, isrEncoder3, isrEncoder4};

  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(encPinsA[i], INPUT_PULLUP);
    pinMode(encPinsB[i], INPUT_PULLUP);

    const int intNum = digitalPinToInterrupt(encPinsA[i]);
    Serial.print(F("Encoder ")); Serial.print(i + 1); Serial.print(F(" pin A: "));
    Serial.print(encPinsA[i]); Serial.print(F(" -> Int: "));
    if (intNum == NOT_AN_INTERRUPT) {
      Serial.println(F("NINGUNA [ERROR CRITICO]"));
    } else {
      Serial.println(intNum);
      attachInterrupt(intNum, encIsrs[i], RISING);
    }
  }

  printMenu();
}

void loop() {
  printTelemetry();

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    while (Serial.available() > 0) Serial.read();

    switch (cmd) {
      case 'r': case 'R': runDeadbandRamp(); break;
      case 'd': case 'D': runDirectionTest(); break;
      case 's': case 'S': stopAllMotors(); Serial.println(F("[MOTORES DETENIDOS]")); break;
      case 'z': case 'Z':
        noInterrupts(); encTicks[0] = encTicks[1] = encTicks[2] = encTicks[3] = 0; interrupts();
        Serial.println(F("[ENCODERS REINICIADOS A CERO]"));
        break;
      case 'm': case 'M': printMenu(); break;
      default: break;
    }
  }
  yield();
}
