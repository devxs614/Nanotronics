// Prueba 4: PWM
// Puede observarse con un LED + resistencia de 220-330 ohm.
// NO conectar un motor directamente al Arduino.

const byte PWM_PIN = 6;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  for (int pwm = 0; pwm <= 255; pwm += 5) {
    analogWrite(PWM_PIN, pwm);
    Serial.println(pwm);
    delay(50);
  }

  for (int pwm = 255; pwm >= 0; pwm -= 5) {
    analogWrite(PWM_PIN, pwm);
    Serial.println(pwm);
    delay(50);
  }
}
