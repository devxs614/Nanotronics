#include "ColorSensor.h"

void ColorSensor::begin() {
  pinMode(TCS_POWER, OUTPUT);
  digitalWrite(TCS_POWER, HIGH);
  delay(TCS_POWER_SETTLE_MS);
  initialized_ = tcs_.begin(TCS34725_I2C_ADDRESS);

  candidateColor_ = robot::ColorClass::COLOR_NONE;
  consistentSamples_ = 0;
  latestReading.r = latestReading.g = latestReading.b = latestReading.c = 0;
  latestReading.rn = latestReading.gn = latestReading.bn = 0.0f;
  latestReading.color = initialized_
    ? robot::ColorClass::COLOR_NONE : robot::ColorClass::COLOR_UNKNOWN;
  latestReading.confidence = 0.0f;
}

void ColorSensor::update() {
  if (!initialized_) {
    latestReading.color = robot::ColorClass::COLOR_UNKNOWN;
    latestReading.confidence = 0.0f;
    return;
  }

  readRaw();
  calculateNormalizedRGB();
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
}

void ColorSensor::readRaw() {
  if (initialized_) {
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