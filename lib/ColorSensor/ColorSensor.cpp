#include "ColorSensor.h"

namespace {
// Layout guardado en la EEPROM (direccion 0) por tests_2.0/09_test_color_calibration_eeprom.cpp
constexpr uint32_t kCalibMagic = 0x434C5233UL;  // "CLR3"

struct StoredSignature {
  uint16_t r, g, b, c;
  float rn, gn, bn;
  float threshold;
  uint8_t valid;
};

struct StoredCalibration {
  uint32_t magic;
  StoredSignature colors[ColorSensor::PROFILE_COUNT];
};

const char* const kShortNames[ColorSensor::PROFILE_COUNT] = {
  "Cian", "Amar", "Naran", "Rosa", "Rojo", "Verde", "Piso"
};
}  // namespace

void ColorSensor::begin() {
  selectI2CChannel(MUX_CH_COLOR);
  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, HIGH);
  delay(TCS_POWER_SETTLE_MS);
  initialized_ = false;
  if (tcs_.begin(TCS34725_I2C_ADDRESS)) initialized_ = true;

  candidateColor_ = robot::ColorClass::COLOR_NONE;
  consistentSamples_ = 0;
  currentSlot_ = SLOT_NONE;
  candidateSlot_ = SLOT_NONE;
  latestReading.r = latestReading.g = latestReading.b = latestReading.c = 0;
  latestReading.rn = latestReading.gn = latestReading.bn = 0.0f;
  latestReading.color = initialized_
    ? robot::ColorClass::COLOR_NONE : robot::ColorClass::COLOR_UNKNOWN;
  latestReading.confidence = 0.0f;

  loadCalibration();
}

void ColorSensor::loadCalibration() {
  memset(&calibrationData_, 0, sizeof(calibrationData_));

  // 1) Formato escrito por el test 09 (codigo de verificacion + firmas por color)
  StoredCalibration stored;
  EEPROM.get(0, stored);
  if (stored.magic == kCalibMagic) {
    robot::ColorProfile* out[PROFILE_COUNT] = {
      &calibrationData_.cian, &calibrationData_.amarillo, &calibrationData_.naranja,
      &calibrationData_.rosa, &calibrationData_.rojo, &calibrationData_.verde,
      &calibrationData_.piso
    };
    for (uint8_t i = 0; i < PROFILE_COUNT; ++i) {
      if (stored.colors[i].valid != 1) continue;
      out[i]->rNorm = stored.colors[i].rn;
      out[i]->gNorm = stored.colors[i].gn;
      out[i]->bNorm = stored.colors[i].bn;
      out[i]->clearRaw = stored.colors[i].c;
      calibrationData_.isCalibrated = true;
    }
    return;
  }

  // 2) Estructura ColorCalibrationData directa en la direccion 0
  EEPROM.get(0, calibrationData_);
  calibrationData_.isCalibrated = false;
  if (calibrationData_.cian.rNorm + calibrationData_.cian.gNorm > 0.1f) {
    bool sane = true;  // EEPROM vacia (0xFF) o datos ajenos producen NaN / valores fuera de [0, 1]
    for (uint8_t i = 0; i < PROFILE_COUNT; ++i) {
      const robot::ColorProfile& p = profile(i);
      if (!(p.rNorm >= 0.0f && p.rNorm <= 1.0f && p.gNorm >= 0.0f && p.gNorm <= 1.0f &&
            p.bNorm >= 0.0f && p.bNorm <= 1.0f)) {
        sane = false;
        break;
      }
    }
    calibrationData_.isCalibrated = sane;
  }
  if (!calibrationData_.isCalibrated) memset(&calibrationData_, 0, sizeof(calibrationData_));
}

const robot::ColorProfile& ColorSensor::profile(uint8_t slot) const {
  switch (slot) {
    case SLOT_CYAN:   return calibrationData_.cian;
    case SLOT_YELLOW: return calibrationData_.amarillo;
    case SLOT_ORANGE: return calibrationData_.naranja;
    case SLOT_PINK:   return calibrationData_.rosa;
    case SLOT_RED:    return calibrationData_.rojo;
    case SLOT_GREEN:  return calibrationData_.verde;
    default:          return calibrationData_.piso;
  }
}

int8_t ColorSensor::slotFromClass(robot::ColorClass c) {
  switch (c) {
    case robot::ColorClass::COLOR_CYAN:   return SLOT_CYAN;
    case robot::ColorClass::COLOR_YELLOW: return SLOT_YELLOW;
    case robot::ColorClass::COLOR_ORANGE: return SLOT_ORANGE;
    case robot::ColorClass::COLOR_PINK:   return SLOT_PINK;
    case robot::ColorClass::COLOR_RED:    return SLOT_RED;
    case robot::ColorClass::COLOR_NONE:   return SLOT_NONE;
    default:                              return SLOT_UNKNOWN;
  }
}

robot::ColorClass ColorSensor::classFromSlot(int8_t slot) {
  switch (slot) {
    case SLOT_CYAN:   return robot::ColorClass::COLOR_CYAN;
    case SLOT_YELLOW: return robot::ColorClass::COLOR_YELLOW;
    case SLOT_ORANGE: return robot::ColorClass::COLOR_ORANGE;
    case SLOT_PINK:   return robot::ColorClass::COLOR_PINK;
    case SLOT_RED:    return robot::ColorClass::COLOR_RED;
    case SLOT_FLOOR:
    case SLOT_NONE:   return robot::ColorClass::COLOR_NONE;
    default:          return robot::ColorClass::COLOR_UNKNOWN;  // Verde y desconocido
  }
}

const char* ColorSensor::getColorName() const {
  if (currentSlot_ >= 0 && currentSlot_ < PROFILE_COUNT) return kShortNames[currentSlot_];
  if (currentSlot_ == SLOT_NONE) return "---";
  return "Desc";
}

// Vecino mas cercano: distancia euclidiana en RGB normalizado + peso del brillo (Clear)
int8_t ColorSensor::classifyCalibrated() {
  const float sum = static_cast<float>(latestReading.r) + latestReading.g + latestReading.b;
  latestReading.confidence = 0.0f;
  if (sum <= 0.0f) return SLOT_UNKNOWN;

  const float rn = static_cast<float>(latestReading.r) / sum;
  const float gn = static_cast<float>(latestReading.g) / sum;
  const float bn = static_cast<float>(latestReading.b) / sum;
  const int c = static_cast<int>(latestReading.c);

  auto calcDist = [&](const robot::ColorProfile& p) -> float {
    const float dRGB = sqrt(sq(rn - p.rNorm) + sq(gn - p.gNorm) + sq(bn - p.bNorm));
    const float dClear = abs(c - static_cast<int>(p.clearRaw)) * 0.002f;
    return dRGB + dClear;
  };

  int8_t best = SLOT_UNKNOWN;
  float bestDist = 1.0e9f;
  for (uint8_t i = 0; i < PROFILE_COUNT; ++i) {
    const robot::ColorProfile& p = profile(i);
    if (p.rNorm + p.gNorm + p.bNorm <= 0.0f) continue;  // Perfil sin muestra
    const float d = calcDist(p);
    if (d < bestDist) {
      bestDist = d;
      best = i;
    }
  }
  if (best != SLOT_UNKNOWN) latestReading.confidence = 1.0f;
  return best;
}
void ColorSensor::update() {
  selectI2CChannel(MUX_CH_COLOR);
  if (!initialized_) {
    latestReading.color = robot::ColorClass::COLOR_UNKNOWN;
    latestReading.confidence = 0.0f;
    currentSlot_ = SLOT_UNKNOWN;
    return;
  }

  readRaw();
  calculateNormalizedRGB();

  if (calibrationData_.isCalibrated) {
    const int8_t sample = classifyCalibrated();
    if (sample == candidateSlot_) {
      if (consistentSamples_ < COLOR_CONFIRM_SAMPLES) ++consistentSamples_;
      if (consistentSamples_ >= COLOR_CONFIRM_SAMPLES) {
        currentSlot_ = sample;
        latestReading.color = classFromSlot(sample);
      }
    } else {
      candidateSlot_ = sample;
      consistentSamples_ = 1;
      currentSlot_ = SLOT_UNKNOWN;
      latestReading.color = robot::ColorClass::COLOR_UNKNOWN;
    }
    return;
  }

  const robot::ColorClass sample = calculateColorClassification();
  if (!isValidColorSample()) {
    candidateColor_ = robot::ColorClass::COLOR_NONE;
    consistentSamples_ = 0;
    latestReading.color = robot::ColorClass::COLOR_NONE;
  } else if (sample == candidateColor_) {
    if (consistentSamples_ < COLOR_CONFIRM_SAMPLES) ++consistentSamples_;
    if (consistentSamples_ >= COLOR_CONFIRM_SAMPLES) latestReading.color = sample;
  } else {
    candidateColor_ = sample;
    consistentSamples_ = 1;
    latestReading.color = robot::ColorClass::COLOR_UNKNOWN;
  }
  currentSlot_ = slotFromClass(latestReading.color);
}

void ColorSensor::readRaw() {
  if (initialized_) {
    selectI2CChannel(MUX_CH_COLOR);
    tcs_.getRawData(&latestReading.r, &latestReading.g, &latestReading.b, &latestReading.c);
  }
}

void ColorSensor::readRGB() {
  readRaw();
}

float ColorSensor::calculateNormalizedRGB() {
  if (latestReading.c == 0) {
    latestReading.rn = latestReading.gn = latestReading.bn = 0.0f;
    return 0.0f;
  }
  latestReading.rn = static_cast<float>(latestReading.r) / latestReading.c;
  latestReading.gn = static_cast<float>(latestReading.g) / latestReading.c;
  latestReading.bn = static_cast<float>(latestReading.b) / latestReading.c;
  return latestReading.rn + latestReading.gn + latestReading.bn;
}

robot::ColorClass ColorSensor::calculateColorClassification() {
  if (latestReading.c < COLOR_CLEAR_MIN || latestReading.c == 0) {
    latestReading.confidence = 0.0f;
    return robot::ColorClass::COLOR_NONE;
  }

  const float total = static_cast<float>(latestReading.r) + latestReading.g + latestReading.b;
  if (total <= COLOR_SATURATION_MIN) {
    latestReading.confidence = 0.0f;
    return robot::ColorClass::COLOR_NONE;
  }

  const float r = latestReading.r / total;
  const float g = latestReading.g / total;
  const float b = latestReading.b / total;
  robot::ColorClass detected = robot::ColorClass::COLOR_UNKNOWN;
  float margin = 0.0f;

  if (g > COLOR_CYAN_GREEN_MIN && b > COLOR_CYAN_BLUE_MIN && r < COLOR_CYAN_RED_MAX) {
    detected = robot::ColorClass::COLOR_CYAN;
    margin = min(g, b) - r;
  } else if (r > COLOR_YELLOW_RED_MIN && g > COLOR_YELLOW_GREEN_MIN && b < COLOR_YELLOW_BLUE_MAX) {
    detected = robot::ColorClass::COLOR_YELLOW;
    margin = min(r, g) - b;
  } else if (r > COLOR_ORANGE_RED_MIN && g > COLOR_ORANGE_GREEN_MIN &&
             g < COLOR_ORANGE_GREEN_MAX && b < COLOR_ORANGE_BLUE_MAX) {
    detected = robot::ColorClass::COLOR_ORANGE;
    margin = r - max(g, b);
  } else if (r > COLOR_PINK_RED_MIN && b > COLOR_PINK_BLUE_MIN && g < COLOR_PINK_GREEN_MAX) {
    detected = robot::ColorClass::COLOR_PINK;
    margin = min(r, b) - g;
  } else if (r > COLOR_RED_RED_MIN && g < COLOR_RED_GREEN_MAX && b < COLOR_RED_BLUE_MAX) {
    detected = robot::ColorClass::COLOR_RED;
    margin = r - max(g, b);
  }

  latestReading.confidence = detected == robot::ColorClass::COLOR_UNKNOWN
    ? 0.0f : constrain(0.55f + margin * 2.0f, 0.0f, 1.0f);
  if (detected != robot::ColorClass::COLOR_UNKNOWN &&
      latestReading.confidence < COLOR_CONFIDENCE_MIN) {
    return robot::ColorClass::COLOR_UNKNOWN;
  }
  return detected;
}

bool ColorSensor::isValidColorSample() const {
  return initialized_ && latestReading.c >= COLOR_CLEAR_MIN &&
         (static_cast<uint32_t>(latestReading.r) + latestReading.g + latestReading.b) > COLOR_SATURATION_MIN &&
         latestReading.confidence >= COLOR_CONFIDENCE_MIN;
}