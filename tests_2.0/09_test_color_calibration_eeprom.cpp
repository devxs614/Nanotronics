#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "../src/config.h"
#include "../lib/ColorSensor/ColorSensor.h"

// Indices de las clases calibrables
enum ColorSlot : uint8_t {
  SLOT_CYAN = 0,
  SLOT_YELLOW,
  SLOT_ORANGE,
  SLOT_PINK,
  SLOT_RED,
  SLOT_GREEN,
  SLOT_FLOOR,
  SLOT_LIME,     // Verde Lima: se guarda en su propia direccion EEPROM
  SLOT_COUNT
};

// Clases del bloque EEPROM original (el layout no cambia; Navigator depende de el)
constexpr uint8_t SLOT_STORED_COUNT = SLOT_LIME;

static const char* const kSlotNames[SLOT_COUNT] = {
  "Cian", "Amarillo", "Naranja", "Rosa", "Rojo", "Verde", "Piso", "Verde Lima"
};

struct ColorSignature {
  uint16_t r, g, b, c;   // Promedio crudo
  float rn, gn, bn;      // RGB normalizado (r/(r+g+b), ...)
  float threshold;       // Distancia maxima aceptada para clasificar
  uint8_t valid;         // 1 si la muestra fue capturada
};

struct ColorCalibrationData {
  uint32_t magic;
  ColorSignature colors[SLOT_STORED_COUNT];
};

struct LimeCalibrationData {
  uint32_t magic;
  ColorSignature color;
};

constexpr uint32_t CALIB_MAGIC = 0x434C5233UL;  // "CLR3" (7 clases)
constexpr uint32_t LIME_MAGIC = 0x4C494D45UL;   // "LIME"
constexpr int EEPROM_ADDR = 0;
constexpr int EEPROM_ADDR_LIME = EEPROM_ADDR + sizeof(ColorCalibrationData);
constexpr uint8_t SAMPLES_PER_CAPTURE = 10;

Adafruit_SSD1306 display(128, 64, &Wire, -1);
ColorSensor color;
ColorCalibrationData configData;
LimeCalibrationData limeData;

bool oledOk = false;
uint16_t liveR = 0, liveG = 0, liveB = 0, liveC = 0;
float liveRn = 0, liveGn = 0, liveBn = 0;
int8_t detected = -1;        // Indice de ColorSlot, -1 si no hay calibracion/coincidencia
float detectedDist = 0.0f;

static void normalize(uint16_t r, uint16_t g, uint16_t b, float& rn, float& gn, float& bn) {
  const float total = static_cast<float>(r) + g + b;
  if (total <= 0.0f) {
    rn = gn = bn = 0.0f;
    return;
  }
  rn = r / total;
  gn = g / total;
  bn = b / total;
}

static float distanceSq(float r1, float g1, float b1, float r2, float g2, float b2) {
  const float dr = r1 - r2, dg = g1 - g2, db = b1 - b2;
  return dr * dr + dg * dg + db * db;
}

static ColorSignature& slotSig(uint8_t slot) {
  return (slot == SLOT_LIME) ? limeData.color : configData.colors[slot];
}

static void readColorSensor() {
  selectI2CChannel(1);
  delay(2);
  color.readRaw();
  liveR = color.latestReading.r;
  liveG = color.latestReading.g;
  liveB = color.latestReading.b;
  liveC = color.latestReading.c;
  normalize(liveR, liveG, liveB, liveRn, liveGn, liveBn);
}

// Umbral de cada color = mitad de la distancia al color calibrado mas cercano
static void computeThresholds() {
  for (uint8_t i = 0; i < SLOT_COUNT; ++i) {
    ColorSignature& a = slotSig(i);
    if (!a.valid) continue;
    float minDist = 1.0e9f;
    for (uint8_t j = 0; j < SLOT_COUNT; ++j) {
      if (i == j || !slotSig(j).valid) continue;
      const ColorSignature& b = slotSig(j);
      const float d = sqrt(distanceSq(a.rn, a.gn, a.bn, b.rn, b.gn, b.bn));
      if (d < minDist) minDist = d;
    }
    a.threshold = (minDist < 1.0e8f) ? minDist * 0.5f : 0.25f;
  }
}

// Distancia Euclidiana minima en espacio RGB normalizado
static void classify() {
  detected = -1;
  detectedDist = 0.0f;
  if (liveC < COLOR_CLEAR_MIN) return;

  float best = 1.0e9f;
  for (uint8_t i = 0; i < SLOT_COUNT; ++i) {
    const ColorSignature& s = slotSig(i);
    if (!s.valid) continue;
    const float d = sqrt(distanceSq(liveRn, liveGn, liveBn, s.rn, s.gn, s.bn));
    if (d < best) {
      best = d;
      detected = i;
    }
  }
  if (detected >= 0) {
    detectedDist = best;
    if (best > slotSig(detected).threshold) detected = -2;  // Fuera de umbral
  }
}

static const char* detectedName() {
  if (detected >= 0) return kSlotNames[detected];
  if (detected == -2) return "Desc";
  return "Sin calib";
}

static void captureSample(uint8_t slot) {
  uint32_t sumR = 0, sumG = 0, sumB = 0, sumC = 0;
  for (uint8_t i = 0; i < SAMPLES_PER_CAPTURE; ++i) {
    readColorSensor();
    sumR += liveR;
    sumG += liveG;
    sumB += liveB;
    sumC += liveC;
    delay(30);
  }

  ColorSignature& s = slotSig(slot);
  s.r = sumR / SAMPLES_PER_CAPTURE;
  s.g = sumG / SAMPLES_PER_CAPTURE;
  s.b = sumB / SAMPLES_PER_CAPTURE;
  s.c = sumC / SAMPLES_PER_CAPTURE;
  normalize(s.r, s.g, s.b, s.rn, s.gn, s.bn);
  s.valid = 1;
  computeThresholds();

  Serial.print(F("[MUESTRA] "));
  Serial.print(kSlotNames[slot]);
  Serial.print(F(" -> R:")); Serial.print(s.r);
  Serial.print(F(" G:")); Serial.print(s.g);
  Serial.print(F(" B:")); Serial.print(s.b);
  Serial.print(F(" C:")); Serial.print(s.c);
  Serial.print(F(" | norm ")); Serial.print(s.rn, 3);
  Serial.print(','); Serial.print(s.gn, 3);
  Serial.print(','); Serial.print(s.bn, 3);
  Serial.print(F(" | umbral ")); Serial.println(s.threshold, 3);
}

static void printCalibration() {
  Serial.println(F("--- CALIBRACION ACTUAL ---"));
  for (uint8_t i = 0; i < SLOT_COUNT; ++i) {
    const ColorSignature& s = slotSig(i);
    Serial.print(kSlotNames[i]);
    Serial.print(F(": "));
    if (!s.valid) {
      Serial.println(F("sin muestra"));
      continue;
    }
    Serial.print(F("R:")); Serial.print(s.r);
    Serial.print(F(" G:")); Serial.print(s.g);
    Serial.print(F(" B:")); Serial.print(s.b);
    Serial.print(F(" C:")); Serial.print(s.c);
    Serial.print(F(" | norm ")); Serial.print(s.rn, 3);
    Serial.print(','); Serial.print(s.gn, 3);
    Serial.print(','); Serial.print(s.bn, 3);
    Serial.print(F(" | umbral ")); Serial.println(s.threshold, 3);
  }
  Serial.println(F("--------------------------"));
}

static void saveLimeToEeprom() {
  limeData.magic = LIME_MAGIC;
  EEPROM.put(EEPROM_ADDR_LIME, limeData);
}

static void saveToEeprom() {
  computeThresholds();
  configData.magic = CALIB_MAGIC;
  EEPROM.put(EEPROM_ADDR, configData);
  if (limeData.color.valid) saveLimeToEeprom();
  Serial.println(F("[EEPROM] Calibracion guardada."));
}

// Verde Lima: 10 muestras promediadas y guardado inmediato en su propia direccion
static void calibrateLime() {
  captureSample(SLOT_LIME);
  saveLimeToEeprom();
  const ColorSignature& s = limeData.color;
  Serial.print(F("[EEPROM] Verde Lima guardado correctamente: R:"));
  Serial.print(s.r);
  Serial.print(F(" G:")); Serial.print(s.g);
  Serial.print(F(" B:")); Serial.print(s.b);
  Serial.print(F(" C:")); Serial.println(s.c);
}

static void loadFromEeprom() {
  ColorCalibrationData stored;
  EEPROM.get(EEPROM_ADDR, stored);
  if (stored.magic != CALIB_MAGIC) {
    Serial.println(F("[EEPROM] No hay calibracion valida almacenada."));
  } else {
    configData = stored;
    Serial.println(F("[EEPROM] Calibracion cargada."));
  }

  LimeCalibrationData storedLime;
  EEPROM.get(EEPROM_ADDR_LIME, storedLime);
  if (storedLime.magic == LIME_MAGIC) {
    limeData = storedLime;
    Serial.println(F("[EEPROM] Verde Lima cargado."));
  }
  printCalibration();
}

static void showMenu() {
  Serial.println(F("\n=============================================="));
  Serial.println(F(" TEST 09: CALIBRACION DE COLOR (TCS34725 + EEPROM)"));
  Serial.println(F("=============================================="));
  Serial.println(F("  'c' CIAN  'y' AMARILLO  'o' NARANJA"));
  Serial.println(F("  'p' ROSA  'r' ROJO      'v' VERDE (Inicio)"));
  Serial.println(F("  'f' PISO  'g' VERDE LIMA (EEPROM propia)"));
  Serial.println(F("  'w' guardar en EEPROM   'l' leer de EEPROM"));
  Serial.println(F("  'm' menu\n"));
}

static void renderOled() {
  if (!oledOk) return;
  selectI2CChannel(5);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);

  display.setCursor(0, 0);
  display.print(F("--- CALIB. COLOR ---"));

  display.setCursor(0, 16);
  display.print(F("RGB: "));
  display.print(liveR);
  display.print(F(", "));
  display.print(liveG);
  display.print(F(", "));
  display.print(liveB);

  display.setCursor(0, 32);
  if (detected == SLOT_LIME) {
    display.print(F("COLOR DETECTADO:"));
    display.setCursor(0, 40);
    display.print(F("VERDE LIMA"));
  } else {
    display.print(F("DET: "));
    display.print(detectedName());
  }

  display.setCursor(0, 52);
  display.print(F("c y o p r v g f|w"));
  display.display();
}

void setup() {
  Serial.begin(115200);

  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, HIGH);
  delay(100);

  Wire.begin();
  Wire.setClock(100000UL);

  memset(&limeData, 0, sizeof(limeData));
  memset(&configData, 0, sizeof(configData));
  configData.magic = CALIB_MAGIC;

  selectI2CChannel(5);
  delay(20);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  Serial.print(F("OLED (Canal 5): "));
  Serial.println(oledOk ? F("[OK]") : F("[FALLO]"));

  selectI2CChannel(1);
  delay(20);
  color.begin();
  Serial.println(F("Color TCS34725 (Canal 1): inicializado"));

  showMenu();
}

void loop() {
  static uint32_t lastReadMs = 0;

  if (Serial.available() > 0) {
    const char cmd = Serial.read();
    switch (cmd) {
      case 'c': case 'C': captureSample(SLOT_CYAN);   break;
      case 'y': case 'Y': captureSample(SLOT_YELLOW); break;
      case 'o': case 'O': captureSample(SLOT_ORANGE); break;
      case 'p': case 'P': captureSample(SLOT_PINK);   break;
      case 'r': case 'R': captureSample(SLOT_RED);    break;
      case 'v': case 'V': captureSample(SLOT_GREEN);  break;
      case 'f': case 'F': captureSample(SLOT_FLOOR);  break;
      case 'g': case 'G': calibrateLime();            break;
      case 'w': case 'W': saveToEeprom();             break;
      case 'l': case 'L': loadFromEeprom();           break;
      case 'm': case 'M': showMenu();                 break;
      default: break;
    }
  }

  if (millis() - lastReadMs >= 100UL) {
    lastReadMs = millis();

    readColorSensor();
    classify();
    renderOled();

    Serial.print(F("RGB: "));
    Serial.print(liveR); Serial.print(F(", "));
    Serial.print(liveG); Serial.print(F(", "));
    Serial.print(liveB); Serial.print(F(" (C:"));
    Serial.print(liveC); Serial.print(F(") | DET: "));
    Serial.print(detectedName());
    if (detected >= 0 || detected == -2) {
      Serial.print(F(" (dist "));
      Serial.print(detectedDist, 3);
      Serial.print(')');
    }
    if (detected == SLOT_LIME) Serial.print(F(" | COLOR DETECTADO: VERDE LIMA"));
    Serial.println();
  }
}
