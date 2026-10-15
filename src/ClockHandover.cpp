#include <Arduino.h>

// Replaces the core's weak SystemClock_Config. The clock tree it sets up is the
// stock one for this board (HSI/2 x16 = 64 MHz); what is added is the first
// half, for the case where the FNIRSI bootloader starts this image.
//
// What the bootloader leaves behind is not known: it cannot be read out. If it
// hands over with the PLL already driving the system clock, HAL_RCC_OscConfig
// refuses to touch a PLL in use and the core stops in Error_Handler before
// setup() has latched the power on. So the clock is first put back to its
// reset state, and any interrupt the bootloader left enabled is silenced,
// since this image has no handler for it.
extern "C" void SystemClock_Config(void) {
    // Independent watchdog, about 4 s (LSI 40 kHz / 64, reload 2500), started
    // before anything that can stall and long before setup() latches the power
    // on through PC10. A hang then ends in a reset, PC10 floats and the unit
    // switches off. Without it a stalled image holds its own power and can
    // only be stopped by pulling the battery; that has happened once.
    // loop() reloads it.
    IWDG->KR  = 0x5555;
    IWDG->PR  = 4;
    IWDG->RLR = 2500;
    IWDG->KR  = 0xAAAA;
    IWDG->KR  = 0xCCCC;

    for (uint8_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }

    // The bootloader jumps with interrupts masked: the FNIRSI application has
    // exactly one CPSIE I, in main, and no CPSID anywhere. Left masked, SysTick
    // never counts and the first delay() never returns.
    __enable_irq();

    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
    RCC->CFGR &= ~RCC_CFGR_SW;
    while (RCC->CFGR & RCC_CFGR_SWS) {}
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) {}
    RCC->CIR = 0x009F0000u;
    SystemCoreClockUpdate();

    RCC_OscInitTypeDef osc = {};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    RCC_ClkInitTypeDef clk = {};
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}
