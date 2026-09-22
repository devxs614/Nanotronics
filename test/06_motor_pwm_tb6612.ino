// Prueba 6: motor mediante TB6612FNG
// Ejemplo para un canal.
// PWMA=6, AIN1=24, AIN2=25, STBY=26.
// El motor debe recibir alimentacion desde la fuente del motor,
// nunca directamente desde un pin del Arduino.

const byte PWMA = 6;
const byte AIN1 = 24;
const byte AIN2 = 25;
const byte STBY = 26;

void motorForward(byte speedPWM) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, speedPWM);
}

void motorStop() {
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
  Serial.begin(115200);
}

void loop() {
  Serial.println("Motor: 30% PWM");
  motorForward(77);
  delay(2000);

  motorStop();
  delay(1000);

  Serial.println("Motor: 60% PWM");
  motorForward(153);
  delay(2000);

  motorStop();
  delay(1000);
}
