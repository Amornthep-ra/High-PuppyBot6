#pragma once
#include <stdint.h>
#include <math.h>

static uint32_t highMotionStallMs = 1000;
static uint32_t highMotionTimeoutMs = 30000;

inline uint32_t HighMotionGuardTime(double value) {
  if (!isfinite(value) || value < 100) return 100;
  if (value > 2147483647.0) return 2147483647u;
  return (uint32_t)value;
}
inline void HighIMUSetMotionGuard(double stallMs, double timeoutMs) {
  highMotionStallMs = HighMotionGuardTime(stallMs);
  highMotionTimeoutMs = HighMotionGuardTime(timeoutMs);
}

// Read both encoder counters as one distance-style value.  Use 64-bit
// magnitudes so INT32_MIN is safe before calculating the average.
inline double HighReadAverageEncoderPulse() {
  int64_t m1 = (int32_t)get_pulse_Encoder(1);
  int64_t m2 = (int32_t)get_pulse_Encoder(2);
  if (m1 < 0) m1 = -m1;
  if (m2 < 0) m2 = -m2;
  return ((double)m1 + (double)m2) / 2.0;
}

[[noreturn]] inline void HighMotionFault(const char *reason) {
  ao();
  Serial.println(reason);
  for (;;) delay(100);
}

// Encoder counts are 32-bit. Unwrap each sample before accumulating;
// this also supports motion across a signed counter boundary.
struct HighEncoderMotion {
  int mode = 1;
  uint32_t previous[2] = {0, 0};
  int64_t delta[2] = {0, 0};
  uint32_t lastChange[2] = {0, 0};
  bool commanded[2] = {false, false};
  uint32_t started = 0;
  double progress = 0;
  double mismatch = 0;

  bool selected(int i) const { return mode == 3 || mode == i + 1; }
  static int64_t difference(uint32_t current, uint32_t before) {
    uint32_t bits = current - before;
    return bits <= 0x7fffffffu ? (int64_t)bits : (int64_t)bits - 4294967296LL;
  }
  void begin(int selection) {
    mode = selection;
    if (mode < 1 || mode > 3) HighMotionFault("ENCODER_SELECTION_INVALID");
    init_encoder(mode);
    started = (uint32_t)millis();
    progress = mismatch = 0;
    for (int i = 0; i < 2; ++i) {
      delta[i] = 0;
      commanded[i] = false;
      lastChange[i] = started;
      if (selected(i)) previous[i] = (uint32_t)get_pulse_Encoder(i + 1);
    }
  }
  // Called after applying the actual integer motor commands.
  void commands(int m1, int m2) {
    const int values[2] = {m1, m2};
    uint32_t now = (uint32_t)millis();
    for (int i = 0; i < 2; ++i) {
      bool moving = values[i] != 0;
      if (!moving || !commanded[i]) lastChange[i] = now;
      commanded[i] = moving;
    }
  }
  double sample() {
    uint32_t now = (uint32_t)millis();
    for (int i = 0; i < 2; ++i) {
      if (!selected(i)) continue;
      uint32_t count = (uint32_t)get_pulse_Encoder(i + 1);
      delta[i] += difference(count, previous[i]);
      if (count != previous[i] || !commanded[i]) lastChange[i] = now;
      previous[i] = count;
      // Grace time is excluded from the stall interval.
      if ((uint32_t)(now - started) < 1000u) lastChange[i] = now;
      if (commanded[i] && (uint32_t)(now - started) >= 1000u &&
          (uint32_t)(now - lastChange[i]) >= highMotionStallMs)
        HighMotionFault(i == 0 ? "ENCODER_M1_STALLED" : "ENCODER_M2_STALLED");
    }
    if ((uint32_t)(now - started) >= highMotionTimeoutMs)
      HighMotionFault("MOTION_TIMEOUT");
    double m1 = fabs((double)delta[0]), m2 = fabs((double)delta[1]);
    mismatch = m1 - m2;
    progress = mode == 3 ? (m1 + m2) / 2.0 : (mode == 1 ? m1 : m2);
    return progress;
  }
};
