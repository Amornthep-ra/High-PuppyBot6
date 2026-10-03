#pragma once
#include <atomic>
#include <math.h>

// Only core0 submits resets; core1 owns IMU updates and acknowledges completion.
static std::atomic<bool> highIMUCoreReady{false};
// Set only after loop1 has published its first live yaw reading.
static std::atomic<bool> highIMUYawLive{false};
// Startup stage shown on the SW1 screen: 0 boot, 1 connecting, 2 rear
// color sensor, 3 yaw calibration, 4 first live sample, 5 ready, -1 failed.
static std::atomic<int> highIMUStartupStage{0};
static std::atomic<int> highIMUResetState{0}; // idle, pending, applying, complete
static float highIMUResetTarget = 180.0f;

inline void HighIMU_ServiceReset() {
  if (highIMUCoreReady.load(std::memory_order_acquire)) {
    highIMUYawLive.store(true, std::memory_order_release);
    highIMUStartupStage.store(5, std::memory_order_release);
  }
  int expected = 1;
  if (!highIMUCoreReady.load(std::memory_order_acquire) ||
      !highIMUResetState.compare_exchange_strong(expected, 2, std::memory_order_acq_rel)) return;
  // Preserve the continuous integrator and its lastYaw history; change origin only.
  PuppyBotIMU::Ref_offset_Con = highIMUResetTarget - PuppyBotIMU::continuousYaw;
  Con_yaw_loop1 = highIMUResetTarget;
  highIMUResetState.store(3, std::memory_order_release);
}

inline bool HighIMU_ResetHeadingAndWait(float heading, uint32_t timeoutMs = 5000) {
  if (!isfinite(heading)) return false;
  const uint32_t started = millis();
  while (!highIMUCoreReady.load(std::memory_order_acquire)) {
    if ((uint32_t)(millis() - started) >= timeoutMs) return false;
    delay(1);
  }
  highIMUResetTarget = heading;
  highIMUResetState.store(1, std::memory_order_release);
  while (highIMUResetState.load(std::memory_order_acquire) != 3) {
    if ((uint32_t)(millis() - started) >= timeoutMs) {
      int pending = 1;
      if (highIMUResetState.compare_exchange_strong(pending, 0, std::memory_order_acq_rel)) return false;
      // If core1 is already applying, wait for its immediate acknowledgement.
    }
    delay(1);
  }
  highIMUResetState.store(0, std::memory_order_release);
  return true;
}
