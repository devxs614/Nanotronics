// Prueba 7: lectura basica de encoder
// Ejemplo: canal A del encoder en pin 2.
// El Mega permite interrupciones externas en este pin.

const byte ENCODER_PIN = 2;
volatile unsigned long pulses = 0;

void encoderISR() {
  pulses++;
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_PIN),
    encoderISR,
    RISING
  );

  Serial.println("Encoder iniciado.");
}

void loop() {
  static unsigned long lastTime = 0;

  if (millis() - lastTime >= 1000) {
    noInterrupts();
    unsigned long count = pulses;
    pulses = 0;
    interrupts();

    Serial.print("Pulsos/segundo: ");
    Serial.println(count);

    lastTime = millis();
  }
}
