// ADC1 + TIM1 + DMA1, sampling the BNC input continuously into a circular
// buffer, the way the original firmware does it.
//
//   ADC1 channel 1 on PA1, 1.5 cycles of sample time, ADCCLK = PCLK2 / 2.
//   TIM1 CC1 triggers each conversion, its pulse at mid period.
//   DMA1 channel 1 carries the result, circular, 16 bit, very high priority.
//
// Two ways of getting the halves out. Up to IRQ_MAX_HZ, and if a block
// function has been given, DMA1's half and full transfer interrupts call it
// with each half as it settles, so the decoder sees every sample whatever the
// screen is doing. Above that a half lasts less time than it takes to decode,
// an interrupt per half would leave the main loop nothing, and the flags are
// polled instead with readyHalf(): halves then go by unread while the screen
// is painted, and that is counted as overrun rather than hidden.
//
// The core leaves DMA1_Channel1_IRQHandler as a weak alias of its default
// handler, so defining it here clashes with nothing.

#ifndef ACQUIRE_H
#define ACQUIRE_H

#include <stdint.h>

namespace Acq {

// A half of this is what one screen shows, so the length sets how much of
// the line is visible at once. 1024 samples a half at 100 kS/s is about 98
// bit times, eight characters of 8N1 traffic, which is the sort of span a
// bench scope puts on screen. Going much longer costs RAM the decoder and
// the snapshot also need.
const uint16_t BUF_LEN = 2048;

const uint32_t IRQ_MAX_HZ = 250000;

// Called from the DMA interrupt. `overrun` says the half before went unread.
typedef void (*BlockFn)(const uint16_t* s, uint16_t n, bool overrun);

void     onBlock(BlockFn fn);           // before begin()
bool     interruptDriven();             // true: onBlock is live, do not poll

// Brackets for the main loop to change state the block function also uses.
void     pause();
void     resume();

// Forgets a half that settled while nobody was collecting, after a long
// pause, so that it is not handed over late and counted as an overrun.
void     discard();

void     begin(uint32_t sampleHz);
void     setSampleRate(uint32_t hz);
uint32_t sampleRate();                  // what was actually achieved

// Hands back a half of the buffer once it is no longer being written, or
// nullptr if neither half is ready. `overrun` says a half went by unread.
const uint16_t* readyHalf(uint16_t& n, bool& overrun);

}
#endif
