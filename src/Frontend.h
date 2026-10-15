// The analogue front end ahead of the ADC: the coarse attenuator relay, the
// coupling switch, and the three lines that pick a tap on the 74HC4051.
//
// Eleven sensitivity ranges, 0 to 10. The relay is off for 0..4 and on for
// 5..10, and the mux walks 0..4 and then 0..5 again, which is how the stock
// firmware steps through them.

#ifndef FRONTEND_H
#define FRONTEND_H

#include <stdint.h>

namespace Frontend {

const uint8_t RANGE_MAX = 10;

void    begin();
void    setRange(uint8_t idx);
uint8_t range();
void    setCouplingDC(bool dc);
bool    couplingDC();

// Volts at the ADC pin for one count: Vref over the full 12 bit scale.
float   voltsPerCountAtPin();

// Volts at the BNC for one volt at the ADC pin, for the range in force.
// Every entry starts at 1.0 and is therefore NOT a calibration: the
// attenuator ratios were never measured. Until setGain is called for a
// range, a voltage on the screen is the voltage at the ADC pin, not at the
// probe, and the display says so.
float   gain();
void    setGain(uint8_t idx, float g);
bool    calibrated();

// Convenience: with a known DC level on the BNC reading `counts` above the
// zero level, work out and store the gain for the range in force.
void    calibrateWith(float knownVolts, int32_t counts);

float   countsToVolts(int32_t counts);

}
#endif
