// The settings that are kept over a power cycle: the theme, and how the
// sender exports the buffer.
//
// They live in the last 16 bytes of flash, 0x0800FFF0 to 0x0800FFFF, at the
// end of the page the stock firmware keeps its calibration in (0x0800FC00,
// records at +0x000 and +0x200, 82 bytes each). Four slots of 4 bytes: each
// save goes in the next empty one, and only when all four are used is the
// page erased, with everything else in it written back as it was. This page
// is the only thing in flash this firmware ever erases or writes; the program
// itself may run up to 0x0800FBFF (platformio.ini).

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

namespace Config {

extern uint8_t format;          // Export::RAW, DUMP or IHEX
extern uint8_t block;           // which of the block sizes

// What the sender sends: 0 the received buffer, k the flash area
// Memory::AREA[k - 1]. Not kept: every start is with the buffer.
extern uint8_t source;

const uint8_t NBLOCK = 4;
uint16_t blockSize();           // 0: the whole buffer in one
uint16_t blockSizeAt(uint8_t i);

const uint16_t PAGE_SIZE = 1024;

void load();                    // at start; defaults if nothing valid is there
bool save();                    // only writes if something differs; false: it did not take

}
#endif
