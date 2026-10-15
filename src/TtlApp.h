// Sends what the inspector has received, or a stretch of flash, out on the
// calibration output, as serial at logic levels, 8N1. The output is held
// high, a line at rest, for as long as this screen is up, and goes back to
// 0 V when it is left.
//
// What is sent (Source), how, and in blocks of what size, is set in Settings
// (see Export.h). The screen shows the CRC-32 of what RUN will send, to
// compare with the CRC-32 of what arrived.
//
//   left, right        the speed; for the buffer it starts at the one the
//                      data came in at
//   up, down           everything, or one block of it
//   RUN                send; RUN again stops it
//   MODE, or OK held   back to the menu
//
// The buffer is the inspector's and stays as it is when the inspector is
// left; it is emptied from Settings, on the Buffer row.

#ifndef TTLAPP_H
#define TTLAPP_H

#include "dso152mod.h"
#include <stdint.h>

namespace Ttl {

void enter(DSO152TFT& tft, const char* buf, int len, uint32_t baud);
bool poll(DSO152TFT& tft, const char* buf, int len);     // false: leave
void leave();

}
#endif
