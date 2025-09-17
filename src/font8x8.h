#ifndef FONT8X8_H
#define FONT8X8_H

#include <stdint.h>

// 8x8 bitmap font - ASCII characters 32-127 (96 characters)
// Each character is 8 bytes, 1 byte per row, MSB = left pixel
extern const uint8_t font8x8_basic[96][8];

// Font info
#define FONT8X8_WIDTH  8
#define FONT8X8_HEIGHT 8
#define FONT8X8_FIRST_CHAR 32  // Space character
#define FONT8X8_LAST_CHAR  127 // DEL character
#define FONT8X8_CHAR_COUNT 96

#endif // FONT8X8_H
