// Prueba 2: entrada digital
// Pulsador entre pin 22 y GND.

const byte BUTTON_PIN = 22;
const byte LED_PIN = 13;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  bool pressed = digitalRead(BUTTON_PIN) == LOW;
  digitalWrite(LED_PIN, pressed ? HIGH : LOW);

  Serial.print("Pulsador: ");
  Serial.println(pressed ? "PRESIONADO" : "LIBRE");

  delay(100);
}
