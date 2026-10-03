#pragma once

// Front RGB sensor on Wire; include after puppybot.h. No motion or release commands.
enum RGBColor {
  RGB_UNKNOWN,
  RGB_RED,
  RGB_GREEN,
  RGB_BLUE,
  RGB_YELLOW,
  RGB_WHITE,
  RGB_BLACK
};

struct RGBReading {
  uint16_t red = 0;
  uint16_t green = 0;
  uint16_t blue = 0;
  uint16_t clear = 0;
  bool valid = false;
};

// Reference samples measured on the field at 50 ms / gain 4X.
constexpr uint16_t RGB_MIN_CLEAR = 10;
// Matches the 50.4 ms integration time configured in setup (21 cycles).
constexpr uint16_t RGB_SATURATION = 21504;
struct RGBReference {
  RGBColor color;
  uint16_t red, green, blue, clear;
};

static RGBReference RGB_REFERENCES[] = {
  {RGB_RED,    477, 183, 180, 790},
  {RGB_GREEN,  124, 390, 307, 783},
  {RGB_BLUE,   111, 224, 337, 641},
  {RGB_YELLOW, 721, 775, 356, 1816},
  {RGB_WHITE,  231, 333, 328, 862},
  {RGB_BLACK,   55,  87,  79, 212}
};

// Provisional tolerances: validate using additional field samples.
// Euclidean distance between normalized R/G/B vectors.
static float RGB_MAX_DISTANCE = 0.035f;
// Best distance must be less than 70% of the next-best distance.
static float RGB_SEPARATION_RATIO = 0.70f;

inline void HighRGB_SetTolerance(float distance, float separation) {
  if (distance >= 0.001f && distance <= 1.0f &&
      separation >= 0.01f && separation <= 0.99f) {
    RGB_MAX_DISTANCE = distance;
    RGB_SEPARATION_RATIO = separation;
  }
}

// Reject unusable calibration without replacing the previous reference.
inline bool HighRGB_SetReference(RGBColor color, uint16_t red, uint16_t green,
                                 uint16_t blue, uint16_t clear) {
  if ((uint32_t)red + green + blue == 0 || clear < RGB_MIN_CLEAR ||
      clear >= RGB_SATURATION || red >= RGB_SATURATION ||
      green >= RGB_SATURATION || blue >= RGB_SATURATION) return false;
  for (RGBReference &ref : RGB_REFERENCES) {
    if (ref.color == color) {
      ref = {color, red, green, blue, clear};
      return true;
    }
  }
  return false;
}

inline bool Read_RGB(RGBReading &sample) {
  sample = RGBReading{};
  if (!isTCSFrontReady() || !isConnected(0x29)) return false;

  // One acquisition for all channels, using the existing sensor library.
  tcs.getRawData(&sample.red, &sample.green, &sample.blue, &sample.clear);
  const uint32_t sum = (uint32_t)sample.red + sample.green + sample.blue;
  sample.valid = sum > 0 && sample.clear >= RGB_MIN_CLEAR &&
                 sample.clear < RGB_SATURATION &&
                 sample.red < RGB_SATURATION &&
                 sample.green < RGB_SATURATION &&
                 sample.blue < RGB_SATURATION;
  return sample.valid;
}

// Classifies an already-read sample; does not perform another I2C read.
// Clear is used for signal validity, not as a brightness-only color label.
// White and black participate in matching so they are not forced into a color.
inline RGBColor Check_Color(const RGBReading &sample) {
  if (!sample.valid) return RGB_UNKNOWN;
  const uint32_t sum = (uint32_t)sample.red + sample.green + sample.blue;
  if (sum == 0) return RGB_UNKNOWN;
  const float r = (float)sample.red / sum;
  const float g = (float)sample.green / sum;
  const float b = (float)sample.blue / sum;

  float best = 10.0f;
  float second = 10.0f;
  RGBColor result = RGB_UNKNOWN;

  for (const RGBReference &ref : RGB_REFERENCES) {
    const float total = (float)ref.red + ref.green + ref.blue;
    const float dr = r - ref.red / total;
    const float dg = g - ref.green / total;
    const float db = b - ref.blue / total;
    const float distanceSquared = dr * dr + dg * dg + db * db;
    if (distanceSquared < best) {
      second = best;
      best = distanceSquared;
      result = ref.color;
    } else if (distanceSquared < second) {
      second = distanceSquared;
    }
  }

  if (best > RGB_MAX_DISTANCE * RGB_MAX_DISTANCE ||
      best >= second * RGB_SEPARATION_RATIO * RGB_SEPARATION_RATIO) {
    return RGB_UNKNOWN;
  }
  return result;
}


inline RGBReading &HighRGB_LastSample() {
  static RGBReading sample;
  return sample;
}

inline void HighRGB_Update() { Read_RGB(HighRGB_LastSample()); }
inline bool HighRGB_Valid() { return HighRGB_LastSample().valid; }
inline bool HighRGB_LastIsColor(RGBColor expected) {
  return HighRGB_Valid() && Check_Color(HighRGB_LastSample()) == expected;
}
inline bool HighRGB_IsColor(RGBColor expected) {
  HighRGB_Update();
  return HighRGB_LastIsColor(expected);
}
inline const char *HighRGB_ColorName() {
  if (!HighRGB_Valid()) return "INVALID";
  switch (Check_Color(HighRGB_LastSample())) {
    case RGB_RED: return "RED";
    case RGB_GREEN: return "GREEN";
    case RGB_BLUE: return "BLUE";
    case RGB_YELLOW: return "YELLOW";
    case RGB_WHITE: return "WHITE";
    case RGB_BLACK: return "BLACK";
    default: return "UNKNOWN";
  }
}
inline uint16_t HighRGB_Channel(int channel) {
  const RGBReading &s = HighRGB_LastSample();
  switch (channel) {
    case 0: return s.red;
    case 1: return s.green;
    case 2: return s.blue;
    case 3: return s.clear;
    default: return 0;
  }
}

inline void HighRGB_Begin() {
  static bool initialized = false;
  if (initialized) return;
  if (!isTCSFrontReady()) return;
  tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
  tcs.setGain(TCS34725_GAIN_4X);
  delay(100);
  initialized = true;
}

inline void HighRGB_Value(int x, int y, const char *label, uint16_t value, uint16_t accent) {
  const uint16_t panel = 0x10C3;
  tft_.fillRoundRect(x, y, 76, 30, 4, panel);
  tft_.setTextColor(accent, panel);
  tft_.setTextSize(1);
  tft_.setCursor(x + 6, y + 2);
  tft_.print(label);
  char digits[6];
  snprintf(digits, sizeof(digits), "%u", (unsigned int)value);
  tft_.setTextSize(2);
  tft_.setTextColor(0xFFFF, panel);
  tft_.setCursor(x + 6, y + 12);
  tft_.print(digits);
}

inline void HighRGB_ReadAndShow() {
  HighRGB_Update();
  const RGBReading &sample = HighRGB_LastSample();
  const RGBColor color = Check_Color(sample);
  const char *label = "UNKNOWN";
  uint16_t accent = 0xBDF7;
  switch (color) {
    case RGB_RED: label = "RED"; accent = 0xF986; break;
    case RGB_GREEN: label = "GREEN"; accent = 0x47E8; break;
    case RGB_BLUE: label = "BLUE"; accent = 0x5CFF; break;
    case RGB_YELLOW: label = "YELLOW"; accent = 0xFFE0; break;
    case RGB_WHITE: label = "WHITE"; accent = 0xFFFF; break;
    case RGB_BLACK: label = "BLACK"; accent = 0xBDF7; break;
    default: break;
  }
  if (!sample.valid) { label = "INVALID"; accent = 0xFD20; }
  // Fixed landscape geometry: all text uses the built-in 6x8 font.
  tft_.setRotation(1);
  tft_.setFont(NULL);
  tft_.setTextWrap(false);
  tft_.fillRect(0, 0, 160, 16, 0x0000);
  tft_.setTextSize(1);
  tft_.setTextColor(0xBDF7, 0x0000);
  tft_.setCursor(5, 4);
  tft_.print("LIVE COLOR");
  tft_.setCursor(112, 4);
  tft_.print("50ms 4X");
  tft_.fillRect(0, 16, 160, 50, 0x0000);
  tft_.drawRoundRect(2, 18, 156, 44, 5, accent);
  tft_.setTextSize(3);
  tft_.setTextColor(accent, 0x0000);
  tft_.setCursor((160 - (int)strlen(label) * 18) / 2, 29);
  tft_.print(label);
  tft_.fillRect(0, 66, 160, 2, 0x0000);
  tft_.fillRect(0, 68, 2, 60, 0x0000);
  tft_.fillRect(78, 68, 4, 60, 0x0000);
  tft_.fillRect(158, 68, 2, 60, 0x0000);
  HighRGB_Value(2, 68, "R", sample.red, 0xF986);
  HighRGB_Value(82, 68, "G", sample.green, 0x47E8);
  HighRGB_Value(2, 98, "B", sample.blue, 0x5CFF);
  HighRGB_Value(82, 98, "CLEAR", sample.clear, 0xFFFF);
}
