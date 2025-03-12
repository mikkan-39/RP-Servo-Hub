#include "Utils.h"

void printHexBuf(uint8_t *buf, int len) {
  for (int i = 0; i < len; i++)
    printf("0x%02X%c", buf[i], i < len - 1 ? ' ' : '\n');
}

uint16_t convertFloatToUShort(float value) {
  // Cap the value within the range of -4096 to 4096
  value = std::clamp(value, -4096.0f, 4096.0f);

  // Determine the direction bit (bit 10)
  uint16_t directionBit = (value < 0) ? 0x0400 : 0x0000;

  // Convert value to range 0-1000
  uint16_t magnitude = static_cast<uint16_t>((std::abs(value) / 4096.0f) * 1000.0f);

  if (magnitude < 50 && magnitude > 10)
    magnitude = 50;

  // Ensure magnitude is within bounds
  magnitude = std::min(magnitude, static_cast<uint16_t>(1000));

  // Combine magnitude with direction bit
  return magnitude | directionBit;
}