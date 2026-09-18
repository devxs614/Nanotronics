#include <Arduino.h>

void setup() {
  // Configurar pines de los 4 motores
  int pins[] = {6, 30, 31, 7, 32, 33, 8, 35, 36, 9, 37, 38};
  for (int p : pins) pinMode(p, OUTPUT);

  // M1 (FR): Adelante
  digitalWrite(30, HIGH); digitalWrite(31, LOW); analogWrite(6, 200);

  // M2 (FL): Adelante
  digitalWrite(32, HIGH); digitalWrite(33, LOW); analogWrite(7, 200);

  // M3 (RR): Adelante
  digitalWrite(35, HIGH); digitalWrite(36, LOW); analogWrite(8, 200);

  // M4 (RL): Adelante
  digitalWrite(37, HIGH); digitalWrite(38, LOW); analogWrite(9, 200);
}

void loop() {
  // Mantener encendidos los 4 motores constantemente para medir voltajes
}