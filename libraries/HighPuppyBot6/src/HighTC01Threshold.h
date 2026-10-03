#pragma once

// Function-local storage keeps both calibration channels independent.
inline float* HighTC01ThresholdStorage() {
  static float thresholds[2] = {0.0f, 0.0f};
  return thresholds;
}

inline void HighTC01SetThreshold(unsigned int sensor, float whiteMin, float colorMax) {
  if (sensor >= 2) return;
  HighTC01ThresholdStorage()[sensor] =
      (colorMax >= 0.0f && whiteMin <= 65535.0f && colorMax < whiteMin)
          ? (whiteMin + colorMax) * 0.5f : 0.0f;
}

inline float HighTC01GetThreshold(unsigned int sensor) {
  return sensor < 2 ? HighTC01ThresholdStorage()[sensor] : 0.0f;
}
