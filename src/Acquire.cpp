#include <Arduino.h>
#include "Acquire.h"

namespace Acq {

static volatile uint16_t g_buf[BUF_LEN];
static uint32_t g_sampleHz = 0;
static BlockFn  g_fn  = 0;
static bool     g_irq = false;
static bool     g_up  = false;

// Below the system tick, so millis() keeps counting while a half is decoded.
static const uint32_t IRQ_PRIO = 5;

static void applyMode() {
    if (!g_up) return;
    bool want = g_fn && g_sampleHz <= IRQ_MAX_HZ;
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    if (want) DMA1_Channel1->CCR |=  (DMA_CCR_HTIE | DMA_CCR_TCIE);
    else      DMA1_Channel1->CCR &= ~(DMA_CCR_HTIE | DMA_CCR_TCIE);
    DMA1->IFCR = DMA_IFCR_CGIF1;
    g_irq = want;
    if (want) {
        NVIC_SetPriority(DMA1_Channel1_IRQn, IRQ_PRIO);
        NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    }
}

void onBlock(BlockFn fn) { g_fn = fn; applyMode(); }
bool interruptDriven() { return g_irq; }
void pause()  { if (g_irq) NVIC_DisableIRQ(DMA1_Channel1_IRQn); }
void resume() { if (g_irq) NVIC_EnableIRQ(DMA1_Channel1_IRQn); }

void discard() {
    DMA1->IFCR = DMA_IFCR_CGIF1;
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
}

// TIM1 hangs off APB2. Its clock is PCLK2 when APB2 is undivided, twice it
// otherwise, which is the usual STM32 timer rule.
static uint32_t timerClock() {
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    uint32_t ppre2 = (RCC->CFGR >> 11) & 0x7u;
    return (ppre2 & 0x4u) ? pclk2 * 2u : pclk2;
}

uint32_t sampleRate() { return g_sampleHz; }

void setSampleRate(uint32_t hz) {
    if (hz == 0) return;
    uint32_t tclk = timerClock();
    uint32_t psc  = 0;
    uint32_t div  = (tclk + hz / 2) / hz;
    while (div > 65536u) { psc++; div = (tclk / (psc + 1u) + hz / 2) / hz; }
    if (div < 2u) div = 2u;

    TIM1->PSC  = psc;
    TIM1->ARR  = div - 1u;
    TIM1->CCR1 = div / 2u;              // the pulse sits at mid period
    TIM1->EGR  = TIM_EGR_UG;            // load the shadow registers now
    g_sampleHz = tclk / (psc + 1u) / div;
    applyMode();
}

void begin(uint32_t sampleHz) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_TIM1EN;
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;

    GPIOA->CRL &= ~(0xFu << 4);         // PA1: MODE = 00, CNF = 00, analog in

    RCC->CFGR &= ~RCC_CFGR_ADCPRE;      // ADCCLK = PCLK2 / 2

    ADC1->CR1   = 0;                    // one channel, no scan, no interrupts
    ADC1->SQR1  = 0;                    // L = 0, a single conversion
    ADC1->SQR3  = 1;                    // SQ1 = channel 1 = PA1
    ADC1->SMPR2 = 0;                    // SMP1 = 000 = 1.5 cycles
    ADC1->CR2   = ADC_CR2_EXTTRIG       // EXTSEL = 000 = TIM1 CC1
                | ADC_CR2_DMA;
    ADC1->CR2  |= ADC_CR2_ADON;         // wake the converter
    delayMicroseconds(10);              // tSTAB
    ADC1->CR2  |= ADC_CR2_RSTCAL; while (ADC1->CR2 & ADC_CR2_RSTCAL) {}
    ADC1->CR2  |= ADC_CR2_CAL;    while (ADC1->CR2 & ADC_CR2_CAL)    {}

    DMA1_Channel1->CCR   = 0;
    DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR  = (uint32_t)g_buf;
    DMA1_Channel1->CNDTR = BUF_LEN;
    DMA1_Channel1->CCR   = DMA_CCR_MSIZE_0      // 16 bit either side
                         | DMA_CCR_PSIZE_0
                         | DMA_CCR_MINC
                         | DMA_CCR_CIRC
                         | DMA_CCR_PL_0 | DMA_CCR_PL_1;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    DMA1_Channel1->CCR  |= DMA_CCR_EN;

    // TIM1 is an advanced timer, so MOE has to be set for OC1REF to come out.
    // CC1E alone does not reach PA8: that pin stays a plain output for LCD_RST
    // because nothing puts it in alternate function mode.
    TIM1->CR1   = 0;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;  // PWM 1
    TIM1->CCER  = TIM_CCER_CC1E;
    TIM1->BDTR  = TIM_BDTR_MOE;
    setSampleRate(sampleHz);
    TIM1->CR1  |= TIM_CR1_ARPE | TIM_CR1_CEN;

    ADC1->CR2 |= ADC_CR2_ADON;          // the second write arms the trigger

    g_up = true;
    applyMode();
}

const uint16_t* readyHalf(uint16_t& n, bool& overrun) {
    uint32_t isr = DMA1->ISR;
    bool ht = (isr & DMA_ISR_HTIF1) != 0;
    bool tc = (isr & DMA_ISR_TCIF1) != 0;

    n = BUF_LEN / 2;
    if (tc) {
        // The pointer has wrapped, so the upper half is settled. Both flags
        // up means the lower half came and went without being read.
        overrun = ht;
        DMA1->IFCR = DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1;
        return (const uint16_t*)g_buf + BUF_LEN / 2;
    }
    if (ht) {
        overrun = false;
        DMA1->IFCR = DMA_IFCR_CHTIF1;
        return (const uint16_t*)g_buf;
    }
    overrun = false;
    n = 0;
    return 0;
}

void dispatchFromIrq() {
    uint16_t n;
    bool over;
    const uint16_t* half = readyHalf(n, over);
    if (half && g_fn) g_fn(half, n, over);
}

}

extern "C" void DMA1_Channel1_IRQHandler(void) { Acq::dispatchFromIrq(); }
