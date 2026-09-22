// Prueba 3: lectura analogica
// Potenciometro: 5V - cursor A0 - GND.

const byte ANALOG_PIN = A0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int raw = analogRead(ANALOG_PIN);
  float voltage = raw * (5.0 / 1023.0);

  Serial.print("ADC = ");
  Serial.print(raw);
  Serial.print(" | Voltaje = ");
  Serial.print(voltage, 3);
  Serial.println(" V");

  delay(100);
}
