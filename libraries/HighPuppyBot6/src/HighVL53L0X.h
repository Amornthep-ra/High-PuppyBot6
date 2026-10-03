#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <atomic>
// Vendored Pololu 1.3.1; implementation included once by the generated sketch.
#include <VL53L0X.h>

static VL53L0X highTof;
static bool highTofReady = false; // Core 1 only owns the device and Wire1.
static uint32_t highTofPoll = 0;
static std::atomic<int> highTofDistance{-1};
static std::atomic<uint32_t> highTofTime{0};

static bool HighToF_ReadRegisters(uint8_t reg, uint8_t *out, uint8_t count) {
  Wire1.beginTransmission(0x29);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom((uint8_t)0x29, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) out[i] = Wire1.read();
  return true;
}

// Call only from setup1, after the board has initialized Wire1 on GPIO6/7.
static void HighToF_BeginCore1() {
  highTofDistance.store(-1);
  highTofReady = false;
  uint8_t id = 0;
  if (!HighToF_ReadRegisters(0xC0, &id, 1) || id != 0xEE) return;
  highTof.setBus(&Wire1);
  highTof.setTimeout(100);
  if (!highTof.init() || !highTof.setMeasurementTimingBudget(33000)) return;
  highTof.startContinuous(40);
  highTofReady = highTof.last_status == 0;
}

// Poll readiness without waiting for a measurement, so IMU updates can continue.
static void HighToF_ServiceCore1() {
  if (!highTofReady || (uint32_t)(millis() - highTofPoll) < 5) return;
  highTofPoll = millis();
  uint8_t interrupt = 0;
  if (!HighToF_ReadRegisters(0x13, &interrupt, 1)) {
    highTofDistance.store(-1); return;
  }
  if (!(interrupt & 7)) return;
  uint8_t result[12];
  if (!HighToF_ReadRegisters(0x14, result, sizeof(result))) {
    highTofDistance.store(-1); return;
  }
  highTof.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 1);
  const int mm = (int(result[10]) << 8) | result[11];
  // Device status 11 = range complete. Reject invalid/out-of-range readings.
  const bool valid = ((result[0] >> 3) & 15) == 11 && mm > 0 &&
                     mm < 8190 && highTof.last_status == 0;
  highTofTime.store(millis());
  highTofDistance.store(valid ? mm : -1);
}

// Core 0 reads only the cache. -1 means missing, invalid, or older than 150 ms.
static int HighToF_DistanceMM() {
  const int mm = highTofDistance.load();
  return (uint32_t)(millis() - highTofTime.load()) <= 150 ? mm : -1;
}
static bool HighToF_Valid() { return HighToF_DistanceMM() >= 0; }
static bool HighToF_Compare(int operation, float threshold) {
  const int mm = HighToF_DistanceMM();
  if (mm < 0) return false;
  switch (operation) {
    case 0: return mm < threshold;
    case 1: return mm <= threshold;
    case 2: return mm > threshold;
    case 3: return mm >= threshold;
    default: return false;
  }
}
