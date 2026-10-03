#pragma once

// One-line text helpers for the 160 x 128 PuppyBot6 TFT.  The built-in font
// is 6 x 8 pixels at size 1 and 12 x 16 pixels at size 2.
enum HighDisplayPosition : uint8_t {
  HIGH_DISPLAY_POS_TOP,
  HIGH_DISPLAY_POS_CENTER,
  HIGH_DISPLAY_POS_BOTTOM
};

enum HighDisplayAlign : uint8_t {
  HIGH_DISPLAY_LEFT,
  HIGH_DISPLAY_CENTER,
  HIGH_DISPLAY_RIGHT
};

inline uint8_t HighDisplayTextSize(const String &text, uint8_t requestedSize) {
  if (requestedSize == 1 || requestedSize == 2) return requestedSize;
  return text.length() <= 13 ? 2 : 1;
}

inline String HighDisplaySingleLine(const String &text, uint8_t size) {
  String line = text;
  line.replace('\n', ' ');
  line.replace('\r', ' ');
  const int maxChars = 160 / (6 * size);
  if (line.length() > maxChars) line.remove(maxChars);
  return line;
}

inline int HighDisplayAlignedX(int chars, uint8_t size, HighDisplayAlign align) {
  const int width = chars * 6 * size;
  if (align == HIGH_DISPLAY_RIGHT) return 160 - width;
  if (align == HIGH_DISPLAY_CENTER) return (160 - width) / 2;
  return 0;
}

inline void HighDisplayPrintLine(const String &text, int y,
                                 HighDisplayAlign align, uint8_t requestedSize,
                                 uint16_t color) {
  const uint8_t size = HighDisplayTextSize(text, requestedSize);
  const String line = HighDisplaySingleLine(text, size);
  const int lineHeight = 8 * size;
  y = constrain(y, 0, 128 - lineHeight);

  tft_.setFont(NULL);
  tft_.setTextWrap(false);
  tft_.setTextSize(size);
  tft_.fillRect(0, y, 160, lineHeight, 0x0000);
  tft_.setTextColor(color, 0x0000);
  tft_.setCursor(HighDisplayAlignedX(line.length(), size, align), y);
  tft_.print(line);
}

inline void HighDisplayTextAt(const String &text, HighDisplayPosition position,
                              HighDisplayAlign align, uint8_t requestedSize,
                              uint16_t color = 0xFFFF) {
  const uint8_t size = HighDisplayTextSize(text, requestedSize);
  const int lineHeight = 8 * size;
  int y = 0;
  if (position == HIGH_DISPLAY_POS_CENTER) y = (128 - lineHeight) / 2;
  if (position == HIGH_DISPLAY_POS_BOTTOM) y = 128 - lineHeight;
  HighDisplayPrintLine(text, y, align, size, color);
}

inline void HighDisplayTextRow(const String &text, int row,
                               HighDisplayAlign align, uint8_t requestedSize,
                               uint16_t color = 0xFFFF) {
  const uint8_t size = HighDisplayTextSize(text, requestedSize);
  const int maxRow = (128 / (8 * size)) - 1;
  row = constrain(row, 0, maxRow);
  HighDisplayPrintLine(text, row * 8 * size, align, size, color);
}

inline void HighDisplayTextXY(const String &text, int x, int y,
                              uint8_t requestedSize,
                              uint16_t color = 0xFFFF) {
  const uint8_t size = HighDisplayTextSize(text, requestedSize);
  const int lineHeight = 8 * size;
  x = constrain(x, 0, 159);
  y = constrain(y, 0, 128 - lineHeight);

  String line = HighDisplaySingleLine(text, size);
  const int maxChars = (160 - x) / (6 * size);
  if (line.length() > maxChars) line.remove(maxChars);

  tft_.setFont(NULL);
  tft_.setTextWrap(false);
  tft_.setTextSize(size);
  tft_.fillRect(x, y, 160 - x, lineHeight, 0x0000);
  tft_.setTextColor(color, 0x0000);
  tft_.setCursor(x, y);
  tft_.print(line);
}
