#include <Arduino.h>
#include <QTRSensors.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>

// Definición de pines XSHUT según la hoja de especificaciones
#define XSHUT_FRONT 24
#define XSHUT_RIGHT 22
#define XSHUT_LEFT  26

// Direcciones I2C exclusivas para liberar el 0x29
#define ADDR_FRONT 0x30
#define ADDR_RIGHT 0x31
#define ADDR_LEFT  0x32

// Instancias de los sensores
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft  = Adafruit_VL53L0X();

// TCS34725 mantendrá la dirección 0x29 nativa
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // PASO 1: Apagar todos los VL53L0X (Reset a estado LOW)
  pinMode(XSHUT_FRONT, OUTPUT); digitalWrite(XSHUT_FRONT, LOW);
  pinMode(XSHUT_RIGHT, OUTPUT); digitalWrite(XSHUT_RIGHT, LOW);
  pinMode(XSHUT_LEFT,  OUTPUT); digitalWrite(XSHUT_LEFT,  LOW);
  delay(10);

  Serial.println("=== INICIALIZANDO BUS I2C UNIFICADO ===");

  // PASO 2: Levantar y re-direccionar Sensor Frontal a 0x30
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(10);
  if (!loxFront.begin(ADDR_FRONT)) {
    Serial.println("Error: VL53L0X Frontal (0x30) no respondio");
  }

  // PASO 3: Levantar y re-direccionar Sensor Derecho a 0x31
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if (!loxRight.begin(ADDR_RIGHT)) {
    Serial.println("Error: VL53L0X Derecho (0x31) no respondio");
  }

  // PASO 4: Levantar y re-direccionar Sensor Izquierdo a 0x32
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  if (!loxLeft.begin(ADDR_LEFT)) {
    Serial.println("Error: VL53L0X Izquierdo (0x32) no respondio");
  }

  // PASO 5: Ahora que nadie compite por 0x29, inicializar TCS34725
  if (tcs.begin(0x29)) {
    Serial.println("ÉXITO: TCS34725 inicializado en 0x29!");
  } else {
    Serial.println("Error: TCS34725 no encontrado en 0x29");
  }
}

void loop() {
  // Lecturas de distancia
  VL53L0X_RangingMeasurementData_t mFront, mRight, mLeft;
  loxFront.rangingTest(&mFront, false);
  loxRight.rangingTest(&mRight, false);
  loxLeft.rangingTest(&mLeft, false);

  // Lectura de color
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Imprimir estado integrado
  Serial.print("Dist [F:"); Serial.print(mFront.RangeMilliMeter);
  Serial.print(" R:"); Serial.print(mRight.RangeMilliMeter);
  Serial.print(" L:"); Serial.print(mLeft.RangeMilliMeter);
  Serial.print("] mm | Color [R:"); Serial.print(r);
  Serial.print(" G:"); Serial.print(g);
  Serial.print(" B:"); Serial.print(b);
  Serial.println("]");

  delay(200);
}
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  Serial.begin(115200);

  // Configurar como tipo Analógico (QTR-8A) en los pines A0-A7
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  Serial.println("=== TEST QTR-8A (CON LIBRERÍA QTRSENSORS) ===");
}

void loop() {
  // En v4.x no se pasa QTR_EMITTERS_ON, solo el arreglo de valores
  qtr.read(sensorValues);

  Serial.print("QTR A0-A7: ");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(100);
}