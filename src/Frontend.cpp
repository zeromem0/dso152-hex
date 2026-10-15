#include <Arduino.h>
#include "Frontend.h"

#define PIN_RELAY    PA2
#define PIN_COSMO    PA3
#define PIN_SENSEL0  PA4
#define PIN_SENSEL1  PA5
#define PIN_SENSEL2  PA6

namespace Frontend {

// Range 10 turned a real RS232 swing into about a hundred counts, which is
// too close to the noise. The original firmware's scale table puts range 7
// eight times more sensitive, where +-6 V should come to about +-400 counts
// and +-12 V still fits.
static uint8_t g_range = 7;
static bool    g_dc    = true;
static float   g_gain[RANGE_MAX + 1];

void begin() {
    pinMode(PIN_RELAY,   OUTPUT);
    pinMode(PIN_COSMO,   OUTPUT);
    pinMode(PIN_SENSEL0, OUTPUT);
    pinMode(PIN_SENSEL1, OUTPUT);
    pinMode(PIN_SENSEL2, OUTPUT);
    for (uint8_t i = 0; i <= RANGE_MAX; i++) g_gain[i] = 1.0f;
    setCouplingDC(true);
    setRange(g_range);
}

void setRange(uint8_t idx) {
    if (idx > RANGE_MAX) idx = RANGE_MAX;
    g_range = idx;
    uint8_t tap = (idx < 5) ? idx : (uint8_t)(idx - 5);
    digitalWrite(PIN_RELAY,   idx < 5 ? LOW : HIGH);
    digitalWrite(PIN_SENSEL0, (tap     ) & 1);
    digitalWrite(PIN_SENSEL1, (tap >> 1) & 1);
    digitalWrite(PIN_SENSEL2, (tap >> 2) & 1);
}

uint8_t range() { return g_range; }

void setCouplingDC(bool dc) { g_dc = dc; digitalWrite(PIN_COSMO, dc ? HIGH : LOW); }
bool couplingDC() { return g_dc; }

float voltsPerCountAtPin() { return 3.3f / 4096.0f; }

float gain() { return g_gain[g_range]; }
void  setGain(uint8_t idx, float g) { if (idx <= RANGE_MAX && g > 0) g_gain[idx] = g; }
bool  calibrated() { return g_gain[g_range] != 1.0f; }

void calibrateWith(float knownVolts, int32_t counts) {
    if (counts == 0) return;
    float atPin = counts * voltsPerCountAtPin();
    if (atPin != 0.0f) setGain(g_range, knownVolts / atPin);
}

float countsToVolts(int32_t counts) {
    return counts * voltsPerCountAtPin() * g_gain[g_range];
}

}
