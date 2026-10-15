#include <Arduino.h>
#include "Config.h"
#include "Theme.h"
#include "Export.h"

namespace Config {

// The flash page the stock firmware keeps its calibration in, and the last 16
// bytes of it, which it never writes (its two records end at +0x51 and
// +0x251). See Config.h.
static const uint32_t PAGE  = 0x0800FC00u;
static const uint32_t SLOTS = 0x0800FFF0u;
static const uint8_t  NSLOT = 4;                // 4 bytes each
static const uint16_t MARK  = 0xA500;           // high byte of every record
static const uint16_t KEY   = 0xD152;

static const uint16_t BLOCK[NBLOCK] = { 0, 64, 256, 1024 };

uint8_t format = Export::DUMP;
uint8_t block  = 0;
uint8_t source = 0;

uint16_t blockSizeAt(uint8_t i) { return i < NBLOCK ? BLOCK[i] : 0; }
uint16_t blockSize() { return blockSizeAt(block); }

// A record is two half words: the settings, theme in bits 0-2, format in 3-4
// and block in 5-6, under a fixed high byte; then the same xor a key. A slot
// half written (power lost between the two) does not pass, nor does an empty one.
static uint16_t pack() {
    return (uint16_t)(MARK | (Theme::idx & 7) | ((format & 3) << 3) | ((block & 3) << 5));
}

static inline uint16_t half(uint32_t a) { return *(const volatile uint16_t*)a; }
static inline uint32_t slot(uint8_t i) { return SLOTS + 4u * i; }
static bool valid(uint8_t i) {
    uint16_t d = half(slot(i));
    return (d & 0xFF80) == MARK && half(slot(i) + 2) == (uint16_t)(d ^ KEY);
}
static bool empty(uint8_t i) { return half(slot(i)) == 0xFFFF && half(slot(i) + 2) == 0xFFFF; }

// The newest record is the last valid one; -1 if none.
static int8_t newest() {
    for (int8_t i = NSLOT - 1; i >= 0; i--) if (valid((uint8_t)i)) return i;
    return -1;
}

void load() {
    int8_t n = newest();
    if (n < 0) return;
    uint16_t d = half(slot((uint8_t)n));
    uint8_t t = d & 7, f = (d >> 3) & 3, b = (d >> 5) & 3;
    if (t < Theme::count())  Theme::idx = t;
    if (f < Export::NFMT)    format = f;
    if (b < NBLOCK)          block = b;
}

// Waits for the flash controller, for no longer than an erase can take.
static bool settle() {
    uint32_t t0 = millis();
    while (FLASH->SR & FLASH_SR_BSY) {
        if ((millis() - t0) > 200) return false;
    }
    bool ok = !(FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR));
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
    return ok;
}

static bool program(uint32_t a, uint16_t v) {
    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint16_t*)a = v;
    bool ok = settle();
    FLASH->CR &= ~FLASH_CR_PG;
    return ok && half(a) == v;
}

// The page as it was, while it is erased and written back.
static uint16_t keep[PAGE_SIZE / 2];

bool save() {
    uint16_t d = pack();
    int8_t n = newest();
    if (n >= 0 && half(slot((uint8_t)n)) == d) return true;

    // the next slot after anything written, valid or not
    int8_t next = NSLOT;
    while (next > 0 && empty((uint8_t)(next - 1))) next--;

    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }
    bool ok = settle();

    if (ok && next >= NSLOT) {
        // All four used: the page has to be erased, and whatever else is in it
        // (the stock calibration) written back exactly, our slots left empty.
        for (uint16_t i = 0; i < PAGE_SIZE / 2; i++) keep[i] = half(PAGE + 2u * i);
        for (uint8_t i = 0; i < NSLOT * 2; i++) keep[(SLOTS - PAGE) / 2 + i] = 0xFFFF;

        FLASH->CR |= FLASH_CR_PER;
        FLASH->AR  = PAGE;
        FLASH->CR |= FLASH_CR_STRT;
        ok = settle();
        FLASH->CR &= ~FLASH_CR_PER;

        for (uint16_t i = 0; ok && i < PAGE_SIZE / 2; i++)
            if (keep[i] != 0xFFFF) ok = program(PAGE + 2u * i, keep[i]);
        next = 0;
    }
    if (ok) ok = program(slot((uint8_t)next), d);
    if (ok) ok = program(slot((uint8_t)next) + 2, (uint16_t)(d ^ KEY));
    FLASH->CR |= FLASH_CR_LOCK;

    return ok && valid((uint8_t)next);
}

}
