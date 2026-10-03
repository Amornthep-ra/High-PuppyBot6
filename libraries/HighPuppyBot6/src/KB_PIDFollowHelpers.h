#pragma once
// Helper routines for PID follow-line blocks (isolated to avoid touching legacy code)

// Mode for extra movement after detecting crossings (use int for simplicity)
static const int KB_EXTRA_ENC = 0;
static const int KB_EXTRA_TIME = 1;

// Run PID line follow until target pulses (with timeout)
// driveMode: 0=2WD (Run_PID), 1=4WD (Run_PID4DW)
// signedSpeed: positive for forward, negative for backward
// readEnc: function pointer returning current encoder pulses
static inline void follow_line_by_encoder_safe(int driveMode,
                                               int signedSpeed,
                                               float kp,
                                               float kd,
                                               long targetPulses,
                                               long timeoutMs,
                                               long (*readEnc)()) {
  long base = readEnc();
  long target = labs(targetPulses);
  unsigned long start = millis();
  bool timedOut = false;

  while (labs(readEnc() - base) < target) {
    if (timeoutMs > 0 && (millis() - start) >= (unsigned long)timeoutMs) {
      timedOut = true;
      break;
    }
    if (driveMode == 1) {
      Run_PID4DW(signedSpeed, kp, kd);
    } else {
      Run_PID(signedSpeed, kp, kd);
    }
  }
  ao();
  if (!timedOut) delay(50);
}

// Run PID line follow until target pulses, store traveled pulses, optional reset & timeout
static inline void follow_line_by_encoder_store_safe(int driveMode,
                                                     int signedSpeed,
                                                     float kp,
                                                     float kd,
                                                     long targetPulses,
                                                     long timeoutMs,
                                                     bool resetEnc,
                                                     long (*readEnc)(),
                                                     long* outPulses) {
  if (resetEnc) {
    resetEncoder(selection_Encoder);
  }
  long base = readEnc();
  long target = labs(targetPulses);
  unsigned long start = millis();
  bool timedOut = false;

  while (labs(readEnc() - base) < target) {
    if (timeoutMs > 0 && (millis() - start) >= (unsigned long)timeoutMs) {
      timedOut = true;
      break;
    }
    if (driveMode == 1) {
      Run_PID4DW(signedSpeed, kp, kd);
    } else {
      Run_PID(signedSpeed, kp, kd);
    }
  }
  long finalEnc = readEnc();
  long traveled = labs(finalEnc - base);
  ao();
  if (!timedOut) delay(50);
  if (outPulses) *outPulses = traveled;
}

// Follow line counting crossings; extra move by encoder or time; optional reset & timeout
static inline void follow_line_cross_safe(int driveMode,
                                          int signedSpeed,
                                          float kp,
                                          float kd,
                                          long sumThreshold,
                                          int crosses,
                                          long extraVal,
                                          int extraMode, // 0 = encoder, 1 = time
                                          long stopDelay,
                                          long timeoutMs,
                                          bool resetEnc,
                                          long (*readEnc)()) {
  if (resetEnc) {
    resetEncoder(selection_Encoder);
  }
  int count = 0;
  long base = readEnc();
  unsigned long start = millis();
  bool timedOut = false;

  while (count < crosses) {
    if (timeoutMs > 0 && (millis() - start) >= (unsigned long)timeoutMs) {
      timedOut = true;
      break;
    }
    if (driveMode == 1) {
      Run_PID4DW(signedSpeed, kp, kd);
    } else {
      Run_PID(signedSpeed, kp, kd);
    }
    if (Read_sumValue_sensor() > sumThreshold) {
      count++;
      delay(50);  // debounce crossings
    }
  }

  if (!timedOut) {
    if (extraMode == KB_EXTRA_ENC) {
      long base2 = readEnc();
      while (labs(readEnc() - base2) < extraVal) {
        if (timeoutMs > 0 && (millis() - start) >= (unsigned long)timeoutMs) {
          timedOut = true;
          break;
        }
        if (driveMode == 1) {
          Run_PID4DW(signedSpeed, kp, kd);
        } else {
          Run_PID(signedSpeed, kp, kd);
        }
      }
    } else {  // KB_EXTRA_TIME
      unsigned long extraStart = millis();
      while ((millis() - extraStart) < (unsigned long)extraVal) {
        if (timeoutMs > 0 && (millis() - start) >= (unsigned long)timeoutMs) {
          timedOut = true;
          break;
        }
        if (driveMode == 1) {
          Run_PID4DW(signedSpeed, kp, kd);
        } else {
          Run_PID(signedSpeed, kp, kd);
        }
      }
    }
  }

  ao();
  if (!timedOut) delay(stopDelay);
}
