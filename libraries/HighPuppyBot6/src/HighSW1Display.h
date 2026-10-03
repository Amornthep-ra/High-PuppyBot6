#pragma once

inline long HighSW1ClampLong(long value, long low, long high) {
  return value < low ? low : (value > high ? high : value);
}

inline int HighSW1ClampInt(float value) {
  if (!(value >= -999999.0f && value <= 999999.0f)) return 0;
  return (int)HighSW1ClampLong((long)lroundf(value), -999, 999);
}

inline const char *HighSW1IMUStatusText() {
  static char failure[26];
  switch (highIMUStartupStage.load(std::memory_order_acquire)) {
    case -1:
      snprintf(failure, sizeof(failure), "IMU FAIL CODE %d", PuppyBotIMU::imuInitErrorCode);
      return failure;
    case 0: return "IMU WAIT CORE 1";
    case 1: return "IMU CONNECTING";
    case 2: return "IMU BACK SENSOR";
    case 3: return "IMU CALIBRATING";
    case 4: return "IMU FIRST YAW";
    default: return "IMU STARTING";
  }
}

inline void HighSW1CenteredText(const char *text, int x, int width, int y,
                                uint16_t color = 0xFFFF) {
  const int maxChars = width / 6;
  int chars = (int)strlen(text);
  if (chars > maxChars) chars = maxChars;
  tft_.fillRect(x, y, width, 8, 0x0000);
  tft_.setTextColor(color, 0x0000);
  tft_.setCursor(x + (width - chars * 6) / 2, y);
  for (int i = 0; i < chars; ++i) tft_.write((uint8_t)text[i]);
}

inline void HighSW1DrawHeader() {
  char line[27];
  if (!highIMUYawLive.load(std::memory_order_acquire)) {
    HighSW1CenteredText(HighSW1IMUStatusText(), 0, 160, 0, 0xFFE0);
    HighSW1CenteredText("Yaw: waiting", 0, 160, 18, 0x07E0);
  } else {
  snprintf(line, sizeof(line), "IMU P:%+4d R:%+4d Y:%+4d",
           HighSW1ClampInt(pitch_loop1), HighSW1ClampInt(roll_loop1),
           HighSW1ClampInt(Con_yaw_loop1));
  HighSW1CenteredText(line, 0, 160, 0, 0xFFE0);

  const long m1 = HighSW1ClampLong((long)get_pulse_Encoder(1), -99999, 999999);
  const long m2 = HighSW1ClampLong((long)get_pulse_Encoder(2), -99999, 999999);
  snprintf(line, sizeof(line), "ENC M1:%+6ld M2:%+6ld", m1, m2);
  HighSW1CenteredText(line, 0, 160, 9, 0x07FF);
  snprintf(line, sizeof(line), "Snapped Heading: %3d deg", Fix_direction());
  HighSW1CenteredText(line, 0, 160, 18, 0x07E0);
  }
  tft_.drawFastHLine(0, 27, 160, 0x39E7);
}

inline void HighSW1DrawFooter() {
  tft_.drawFastHLine(0, 103, 160, 0x39E7);
  tft_.fillRect(0, 104, 160, 24, 0x0000);
  HighSW1CenteredText("Please press SW1 to start", 0, 160, 111, 0xFFFF);
}

inline void HighSW1Prepare() {
  init_encoder(3);
  pinMode(swpin, INPUT_PULLUP);
  tft_.setRotation(1);
  tft_.setFont(NULL);
  tft_.setTextSize(1);
  tft_.setTextWrap(false);
  tft_.fillScreen(0x0000);
  HighSW1DrawFooter();
}

inline void HighSW1Finish(bool useBuzzer, uint32_t delayAfterPressMs) {
  delay(30);
  if (useBuzzer) buzzer(2000, 100);
  if (delayAfterPressMs > 0) delay(delayAfterPressMs);
}

inline void HighSW1CheckIMUStartup(uint32_t started) {
  if (highIMUYawLive.load(std::memory_order_acquire)) return;
  const bool failed = highIMUStartupStage.load(std::memory_order_acquire) < 0;
  if (!failed && (uint32_t)(millis() - started) < 30000u) return;
  HighSW1CenteredText(failed ? HighSW1IMUStatusText() : "IMU START TIMEOUT",
                      0, 160, 0, 0xF800);
  HighSW1CenteredText(failed ? "CHECK IMU / I2C" : HighSW1IMUStatusText(),
                      0, 160, 18, 0xF800);
  HighMotionFault(failed ? "IMU_INIT_FAILED" : "IMU_START_TIMEOUT");
}

inline void HighWaitSW1AllSensors(bool useBuzzer = true,
                                  uint32_t delayAfterPressMs = 0) {
  HighSW1Prepare();
  const uint32_t started = (uint32_t)millis();
  while (digitalRead(swpin) != LOW || !highIMUYawLive.load(std::memory_order_acquire)) {
    HighSW1CheckIMUStartup(started);
    HighSW1DrawHeader();
    for (int sensor = 0; sensor < 16; ++sensor) {
      const int column = sensor / 8;
      const int row = sensor % 8;
      char value[12];
      snprintf(value, sizeof(value), "A%-2d:%4d", sensor,
               (int)HighSW1ClampLong((long)ADC(sensor), -999, 9999));
      HighSW1CenteredText(value, column * 80, 80, 30 + row * 9);
    }
    delay(25);
  }
  HighSW1Finish(useBuzzer, delayAfterPressMs);
}

inline void HighWaitSW1OneSensor(bool useBuzzer = true,
                                 uint32_t delayAfterPressMs = 0) {
  HighSW1Prepare();
  const uint32_t started = (uint32_t)millis();
  int sensor = 0;
  uint32_t nextSensorAt = millis() + 2000;
  while (digitalRead(swpin) != LOW || !highIMUYawLive.load(std::memory_order_acquire)) {
    HighSW1CheckIMUStartup(started);
    const uint32_t now = millis();
    if ((int32_t)(now - nextSensorAt) >= 0) {
      sensor = (sensor + 1) % 16;
      nextSensorAt += 2000;
    }
    HighSW1DrawHeader();
    char line[22];
    snprintf(line, sizeof(line), "SENSOR A%d", sensor);
    HighSW1CenteredText(line, 0, 160, 38, 0xFFE0);
    snprintf(line, sizeof(line), "VALUE %d",
             (int)HighSW1ClampLong((long)ADC(sensor), -999, 9999));
    HighSW1CenteredText(line, 0, 160, 56);
    snprintf(line, sizeof(line), "%d OF 16", sensor + 1);
    HighSW1CenteredText(line, 0, 160, 74, 0x07FF);
    delay(20);
  }
  HighSW1Finish(useBuzzer, delayAfterPressMs);
}
