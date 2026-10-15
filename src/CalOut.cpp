#include <Arduino.h>
#include "CalOut.h"

namespace CalOut {

static bool g_tone = false;
static bool g_high = false;     // the level at rest

// PA0 is the low nibble of CRL: 0x3 a push-pull output, 0xB the same driven
// by the alternate function, which here is TIM2 channel 1.
static inline void pinGpio() { GPIOA->CRL = (GPIOA->CRL & ~0xFu) | 0x3u; }
static inline void pinTimer() { GPIOA->CRL = (GPIOA->CRL & ~0xFu) | 0xBu; }
static inline void high() { GPIOA->BSRR = 1u; }
static inline void low()  { GPIOA->BRR  = 1u; }

// TIM2 hangs off APB1: PCLK1 when APB1 is undivided, twice it otherwise.
static uint32_t timerClock() {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t ppre1 = (RCC->CFGR >> 8) & 0x7u;
    return (ppre1 & 0x4u) ? pclk1 * 2u : pclk1;
}

void begin() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1 = 0;
    low();
    pinGpio();
}

void level(bool h) {
    g_high = h;
    if (g_tone) { toneOff(); return; }
    if (h) high(); else low();
}

uint16_t send(const uint8_t* p, uint16_t n, uint32_t baud, bool (*go)()) {
    if (!p || baud == 0) return 0;
    toneOff();

    uint16_t sent = 0;
    for (; sent < n; sent++) {
        if (go && !go()) break;

        // start, eight data bits from the low end, stop
        uint16_t frame = (uint16_t)(((uint16_t)p[sent] << 1) | 0x200u);

        // Each edge is timed from the start of the character, not from the
        // edge before, so what a bit runs late is not handed on to the next.
        uint32_t t0 = micros();
        for (uint32_t k = 0; k < 10; k++) {
            if (frame & (1u << k)) high(); else low();
            uint32_t until = ((k + 1u) * 1000000u) / baud;
            while ((uint32_t)(micros() - t0) < until) {}
        }
    }
    if (g_high) high(); else low();
    return sent;
}

void tone(uint32_t hz) {
    if (hz == 0) { toneOff(); return; }
    uint32_t tclk = timerClock();
    uint32_t psc  = 0;
    uint32_t div  = (tclk + hz / 2) / hz;
    while (div > 65536u) { psc++; div = (tclk / (psc + 1u) + hz / 2) / hz; }
    if (div < 2u) div = 2u;

    TIM2->CR1   = 0;
    TIM2->PSC   = psc;
    TIM2->ARR   = div - 1u;
    TIM2->CCR1  = div / 2u;
    TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;  // PWM 1
    TIM2->CCER  = TIM_CCER_CC1E;
    TIM2->EGR   = TIM_EGR_UG;           // load the shadow registers now
    TIM2->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;
    pinTimer();
    g_tone = true;
}

void toneOff() {
    if (!g_tone) return;
    if (g_high) high(); else low();
    pinGpio();
    TIM2->CR1 = 0;
    g_tone = false;
}

bool toneOn() { return g_tone; }

}
