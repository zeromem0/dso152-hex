// The calibration output, PA0, put to two uses.
//
// As a serial line at logic levels: 8N1, at rest high, a one at 3.3 V and a
// zero at 0 V. PA0 is on no USART's transmit pin, so the bits are set by hand
// against micros().
//
// As a square wave, which is what the stock firmware has on this pin. PA0 is
// TIM2 channel 1, so the timer drives it and nothing has to be done while it
// runs.
//
// Otherwise the pin sits at the level it was last told to, 0 V to begin
// with. The sender holds it high for as long as it is on the screen, so that
// a receiver sees an idle line there and not a break.

#ifndef CALOUT_H
#define CALOUT_H

#include <stdint.h>

namespace CalOut {

void begin();

void level(bool high);          // the level at rest; ends a square wave

// Sends n bytes. `go` is asked before each one and stops the sending by
// returning false; it is also the place to keep a watchdog fed, since a full
// buffer at a low speed takes seconds. Returns how many went out.
uint16_t send(const uint8_t* p, uint16_t n, uint32_t baud, bool (*go)());

void tone(uint32_t hz);         // 50 % square wave, until toneOff()
void toneOff();
bool toneOn();

}
#endif
