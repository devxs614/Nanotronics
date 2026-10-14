#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "../src/config.h"
#include "../lib/EncoderManager/EncoderManager.h"
#include "../lib/MotorController/MotorController.h"

Adafruit_SSD1306 display(128, 64, &Wire, -1);
EncoderManager encoderManager;
MotorController motorController;

const float DISTANCIA_PRUEBA_MM = 500.0f;

bool oledOk = false;
int32_t enc1_current = 0;
int32_t enc2_current = 0;
int32_t enc3_current = 0;
int32_t enc4_current = 0;
float ticksPerMm = 0.0f;

extern void selectI2CChannel(uint8_t channel);

void initOled() {
  selectI2CChannel(5);
  delay(20);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOk) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10);
    display.println(F("  CALIB ODOMETRIA  "));
    display.display();
  }
}

void renderOled() {
  if (!oledOk) return;
  selectI2CChannel(5);
  delay(2);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Fila 1: Título
  display.setCursor(0, 0);
  display.println(F("-- CALIB ODOMETRIA --"));

  // Fila 2: E1 y E2
  display.setCursor(0, 16);
  display.print(F("E1:"));
  display.print(enc1_current);
  display.print(F("  E2:"));
  display.println(enc2_current);

  // Fila 3: E3 y E4
  display.setCursor(0, 32);
  display.print(F("E3:"));
  display.print(enc3_current);
  display.print(F("  E4:"));
  display.println(enc4_current);

  // Fila 4: TICKS_PER_MM
  display.setCursor(0, 48);
  display.print(F("TK/MM:"));
  display.println(ticksPerMm, 2);

  display.display();
}

void printMenu() {
  Serial.println(F("\n=============================================="));
  Serial.println(F(" TEST 10: CALIBRACIÓN DE ODOMETRÍA (TESTS 2.0)"));
  Serial.println(F("=============================================="));
  Serial.println(F("Comandos disponibles:"));
  Serial.println(F("  'r' : Reiniciar contadores de encoders"));
  Serial.println(F("  'm' : Modo Manual (Avanzar 50 cm a mano)"));
  Serial.println(F("  'a' : Modo Automático (PWM 90, 1.5 seg)"));
  Serial.println(F("  's' : Detener motores"));
  Serial.println(F("  '?' : Mostrar este menu"));
  Serial.println(F("==============================================\n"));
}

void resetEncoders() {
  encoderManager.resetTicks();
  enc1_current = 0;
  enc2_current = 0;
  enc3_current = 0;
  enc4_current = 0;
  ticksPerMm = 0.0f;
  Serial.println(F("[ENCODERS REINICIADOS A CERO]"));
}

void manualMode() {
  Serial.println(F("\n--- MODO MANUAL ---"));
  Serial.println(F("Desplaza el robot 50 cm sobre una regla."));
  Serial.println(F("Presiona ENTER cuando hayas terminado..."));

  while (Serial.available()) Serial.read();
  while (!Serial.available()) {
    delay(50);
    enc1_current = encoderManager.getTicks(0);
    enc2_current = encoderManager.getTicks(1);
    enc3_current = encoderManager.getTicks(2);
    enc4_current = encoderManager.getTicks(3);
    renderOled();
  }
  while (Serial.available()) Serial.read();

  // Calcular promedio de ticks
  int32_t abs1 = (enc1_current >= 0) ? enc1_current : -enc1_current;
  int32_t abs2 = (enc2_current >= 0) ? enc2_current : -enc2_current;
  int32_t abs3 = (enc3_current >= 0) ? enc3_current : -enc3_current;
  int32_t abs4 = (enc4_current >= 0) ? enc4_current : -enc4_current;

  float promedio = static_cast<float>(abs1 + abs2 + abs3 + abs4) / 4.0f;
  ticksPerMm = promedio / DISTANCIA_PRUEBA_MM;

  Serial.println(F("\n--- RESULTADOS MODO MANUAL ---"));
  Serial.print(F("E1 (abs): ")); Serial.println(abs1);
  Serial.print(F("E2 (abs): ")); Serial.println(abs2);
  Serial.print(F("E3 (abs): ")); Serial.println(abs3);
  Serial.print(F("E4 (abs): ")); Serial.println(abs4);
  Serial.print(F("Promedio: ")); Serial.println(promedio, 2);
  Serial.print(F("TICKS_PER_MM = ")); Serial.print(promedio, 2);
  Serial.print(F(" / ")); Serial.print(DISTANCIA_PRUEBA_MM, 1);
  Serial.print(F(" = ")); Serial.println(ticksPerMm, 4);
}

void automaticMode() {
  Serial.println(F("\n--- MODO AUTOMÁTICO ---"));
  Serial.println(F("Robot avanzará a PWM 90 durante 1.5 segundos..."));

  resetEncoders();
  delay(200);

  motorController.setSpeeds(90, 90, 90, 90);
  uint32_t startMs = millis();

  while (millis() - startMs < 1500UL) {
    delay(50);
    motorController.update();
    enc1_current = encoderManager.getTicks(0);
    enc2_current = encoderManager.getTicks(1);
    enc3_current = encoderManager.getTicks(2);
    enc4_current = encoderManager.getTicks(3);
    renderOled();
  }

  motorController.stop();
  delay(100);

  enc1_current = encoderManager.getTicks(0);
  enc2_current = encoderManager.getTicks(1);
  enc3_current = encoderManager.getTicks(2);
  enc4_current = encoderManager.getTicks(3);

  // Calcular promedio de ticks
  int32_t abs1 = (enc1_current >= 0) ? enc1_current : -enc1_current;
  int32_t abs2 = (enc2_current >= 0) ? enc2_current : -enc2_current;
  int32_t abs3 = (enc3_current >= 0) ? enc3_current : -enc3_current;
  int32_t abs4 = (enc4_current >= 0) ? enc4_current : -enc4_current;

  float promedio = static_cast<float>(abs1 + abs2 + abs3 + abs4) / 4.0f;
  
  // Estimación distancia = promedio / ticks_per_rev * circunferencia
  // O usar la distancia conocida si se mide por tiempo/PWM
  // Aquí asumimos 500 mm teóricos basado en tiempo de ejecución
  ticksPerMm = promedio / DISTANCIA_PRUEBA_MM;

  Serial.println(F("\n--- RESULTADOS MODO AUTOMÁTICO ---"));
  Serial.print(F("E1 (abs): ")); Serial.println(abs1);
  Serial.print(F("E2 (abs): ")); Serial.println(abs2);
  Serial.print(F("E3 (abs): ")); Serial.println(abs3);
  Serial.print(F("E4 (abs): ")); Serial.println(abs4);
  Serial.print(F("Promedio: ")); Serial.println(promedio, 2);
  Serial.print(F("TICKS_PER_MM = ")); Serial.print(promedio, 2);
  Serial.print(F(" / ")); Serial.print(DISTANCIA_PRUEBA_MM, 1);
  Serial.print(F(" = ")); Serial.println(ticksPerMm, 4);
  Serial.println(F("(Nota: Distancia teórica ~500 mm basada en PWM/tiempo)"));
}

void printTelemetry() {
  static uint32_t lastMs = 0;
  if (millis() - lastMs >= 100UL) {
    lastMs = millis();
    Serial.print(F("[TELEMETRÍA] E1:"));
    Serial.print(enc1_current);
    Serial.print(F(" E2:"));
    Serial.print(enc2_current);
    Serial.print(F(" E3:"));
    Serial.print(enc3_current);
    Serial.print(F(" E4:"));
    Serial.print(enc4_current);
    Serial.print(F(" | TICKS_PER_MM:"));
    Serial.println(ticksPerMm, 4);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000UL);

  initOled();
  motorController.begin();
  encoderManager.begin();
  motorController.stop();

  Serial.println(F("\n[ODOMETRÍA CALIBRATION READY]"));
  printMenu();
}

void loop() {
  enc1_current = encoderManager.getTicks(0);
  enc2_current = encoderManager.getTicks(1);
  enc3_current = encoderManager.getTicks(2);
  enc4_current = encoderManager.getTicks(3);

  renderOled();
  printTelemetry();

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    while (Serial.available() > 0) Serial.read();

    switch (cmd) {
      case 'r': case 'R':
        resetEncoders();
        break;
      case 'm': case 'M':
        manualMode();
        break;
      case 'a': case 'A':
        automaticMode();
        break;
      case 's': case 'S':
        motorController.stop();
        Serial.println(F("[MOTORES DETENIDOS]"));
        break;
      case '?':
        printMenu();
        break;
      default:
        break;
    }
  }

  delay(10);
}
