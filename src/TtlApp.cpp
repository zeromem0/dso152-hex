#include <Arduino.h>
#include "TtlApp.h"
#include "Theme.h"
#include "Keys.h"
#include "CalOut.h"
#include "Export.h"
#include "Config.h"
#include "MemApp.h"

namespace Ttl {

// What is sent: the received buffer, or the stretch of flash chosen in
// Settings (Source). Everything below works on these.
static const char* g_buf;
static int         g_len;
static const Memory::Area* g_area;      // null: the buffer

static void pick(const char* buf, int len) {
    if (Config::source == 0 || Config::source > Memory::NAREA) {
        g_area = 0; g_buf = buf; g_len = len;
    } else {
        g_area = &Memory::AREA[Config::source - 1];
        g_buf = (const char*)g_area->base; g_len = (int)g_area->size;
    }
}

static const uint32_t BAUD[] = { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 };
static const uint8_t  NBAUD  = sizeof(BAUD) / sizeof(BAUD[0]);
static uint8_t  g_idx = 3;      // 9600
static uint16_t g_sel = 0;      // 0: everything; otherwise that block, from 1

#define SIZE_Y     28
#define BAUD_Y     52
#define BLOCK_Y    76
#define STATE_Y    102
#define TAIL_Y     132
#define TAIL_COLS  52
#define TAIL_ROWS  3

// --- the buffer in blocks ---------------------------------------------------
static uint16_t blocks(int len) {
    uint16_t bs = Config::blockSize();
    if (bs == 0 || len <= 0) return 1;
    return (uint16_t)((len + bs - 1) / bs);
}

static void blockAt(uint16_t blk, int len, uint16_t& off, uint16_t& n) {
    uint16_t bs = Config::blockSize();
    if (bs == 0) { off = 0; n = (uint16_t)len; return; }
    off = (uint16_t)((blk - 1) * bs);
    n   = (uint16_t)(len - off < bs ? len - off : bs);
}

// what RUN would send now
static void chosen(int len, uint16_t& off, uint16_t& n) {
    if (g_sel == 0) { off = 0; n = (uint16_t)len; }
    else            blockAt(g_sel, len, off, n);
}

// --- the screen ---------------------------------------------------------------
static void drawSize(DSO152TFT& tft, const char* buf, int len) {
    uint16_t off, n;
    chosen(len, off, n);
    tft.fillRect(0, SIZE_Y, 320, 16, TH.bg);
    tft.setTextSize(2);
    tft.setTextColor(n ? TH.text : TH.muted, TH.bg);
    tft.setCursor(4, SIZE_Y);
    tft.print(n);
    tft.print(" bytes");
    if (n) {
        uint32_t crc = Export::crc32((const uint8_t*)buf + off, n);
        tft.setTextColor(TH.trace, TH.bg);
        tft.setCursor(148, SIZE_Y);
        tft.print("CRC ");
        for (int8_t s = 28; s >= 0; s -= 4) tft.print((char)"0123456789ABCDEF"[(crc >> s) & 0xF]);
    }
}

static void drawBaud(DSO152TFT& tft) {
    tft.fillRect(0, BAUD_Y, 320, 16, TH.bg);
    tft.setTextSize(2);
    tft.setTextColor(TH.text2, TH.bg);
    tft.setCursor(4, BAUD_Y);
    tft.print(BAUD[g_idx]);
    tft.print(" 8N1");
    tft.setCursor(148, BAUD_Y);
    tft.print(Export::name(Config::format));
}

static void drawBlock(DSO152TFT& tft, int len) {
    uint16_t nb = blocks(len);
    tft.fillRect(0, BLOCK_Y, 320, 16, TH.bg);
    tft.setTextSize(2);
    tft.setTextColor(TH.text2, TH.bg);
    tft.setCursor(4, BLOCK_Y);
    if (g_sel == 0) {
        tft.print("ALL");
        if (nb > 1) {
            tft.setCursor(148, BLOCK_Y);
            tft.print(nb); tft.print(" x "); tft.print(Config::blockSize());
        }
    } else {
        uint16_t off, n;
        blockAt(g_sel, len, off, n);
        tft.print("BLOCK "); tft.print(g_sel); tft.print('/'); tft.print(nb);
        tft.setCursor(148, BLOCK_Y);
        tft.print("at "); tft.print(off);
    }
}

// One line of state. `n` is printed after the text when it is not negative.
static void drawState(DSO152TFT& tft, const char* txt, int n, uint16_t col) {
    int16_t len = (int16_t)strlen(txt);
    if (n >= 0) len = (int16_t)(len + 1 + (n >= 1000 ? 4 : n >= 100 ? 3 : n >= 10 ? 2 : 1));
    tft.fillRect(0, STATE_Y, 320, 16, TH.bg);
    tft.setTextSize(2);
    tft.setTextColor(col, TH.bg);
    tft.setCursor((int16_t)((320 - len * 12) / 2), STATE_Y);
    tft.print(txt);
    if (n >= 0) { tft.print(' '); tft.print(n); }
}

static void drawReady(DSO152TFT& tft, int len) {
    if (len) drawState(tft, "READY", -1, TH.ok);
    else     drawState(tft, "BUFFER EMPTY", -1, TH.muted);
}

// The end of what is chosen, as text, so that it can be seen what is about
// to go out. What does not print is shown as a dot.
static void drawTail(DSO152TFT& tft, const char* buf, int len) {
    uint16_t off, n;
    chosen(len, off, n);
    const int shown = TAIL_COLS * TAIL_ROWS;
    int start = n > shown ? n - shown : 0;
    tft.fillRect(0, TAIL_Y, 320, TAIL_ROWS * 12, TH.bg);
    tft.setTextSize(1);
    tft.setTextColor(TH.text, TH.bg);
    for (int i = start; i < n; i++) {
        int k = i - start;
        if (k % TAIL_COLS == 0) tft.setCursor(4, (int16_t)(TAIL_Y + (k / TAIL_COLS) * 12));
        uint8_t b = (uint8_t)buf[off + i];
        tft.print((char)((b >= 32 && b < 127) ? b : '.'));
    }
}

static void drawChoice(DSO152TFT& tft, const char* buf, int len) {
    drawSize(tft, buf, len);
    drawBlock(tft, len);
    drawTail(tft, buf, len);
}

void enter(DSO152TFT& tft, const char* buf0, int len0, uint32_t baud) {
    Keys::sync();
    CalOut::level(true);                // an idle line, for as long as this screen is up
    pick(buf0, len0);
    const char* buf = g_buf;
    int         len = g_len;
    if (!g_area) for (uint8_t i = 0; i < NBAUD; i++) if (BAUD[i] == baud) g_idx = i;
    if (g_sel > blocks(len) || blocks(len) == 1) g_sel = 0;

    tft.fillScreen(TH.bg);
    tft.fillRect(0, 0, 320, 20, TH.bar);
    tft.setTextSize(1);
    tft.setTextColor(TH.barText, TH.bar);
    tft.setCursor(2, 6);
    tft.print("TTL SENDER");
    tft.setCursor(74, 6);
    tft.print("from ");
    tft.print(g_area ? g_area->name : "Buffer");
    tft.setCursor(200, 6);
    tft.print("cal. output 3.3 V");

    drawBaud(tft);
    drawChoice(tft, buf, len);
    drawReady(tft, len);

    tft.drawFastHLine(0, TAIL_Y - 6, 320, TH.line);
    tft.drawFastHLine(0, 196, 320, TH.line);
    tft.setTextSize(1);
    tft.setTextColor(TH.muted, TH.bg);
    tft.setCursor(8, 204);
    tft.print((char)0x11); tft.print((char)0x10);       // left and right arrows
    tft.print(" speed   ");
    tft.print((char)0x1E); tft.print((char)0x1F);       // up and down arrows
    tft.print(" block   RUN send / stop");
    tft.setCursor(8, 220);
    tft.print("MODE menu   format, source, erase: Settings");
}

// --- sending --------------------------------------------------------------------
// Asked before each byte. RUN is still down from the press that started the
// sending, so it has to come up before a press of it can mean stop.
static bool     g_runUp;
static uint32_t g_baud;
static bool keepSending() {
    IWDG->KR = 0xAAAA;
    if (!Keys::down(Keys::RUN)) g_runUp = true;
    else if (g_runUp) return false;
    return true;
}

static bool toWire(const uint8_t* p, uint16_t n) {
    return CalOut::send(p, n, g_baud, keepSending) == n;
}

// A breath between blocks, so that they are apart on the wire and in a log.
static bool pauseBetween() {
    uint32_t t0 = millis();
    while ((millis() - t0) < 150) if (!keepSending()) return false;
    return true;
}

// In the hex dump, a stretch of flash is announced first: the offsets that
// follow count from its start. The receivers pass over '#' lines they do not know.
static bool sendSource() {
    char line[64];
    uint8_t n = 0;
    const char* s = "# DSO152 source ";
    while (*s) line[n++] = *s++;
    for (s = g_area->name; *s; ) line[n++] = *s++;
    line[n++] = ' ';
    for (int8_t k = 28; k >= 0; k -= 4) line[n++] = "0123456789ABCDEF"[(g_area->base >> k) & 0xF];
    s = " length ";
    while (*s) line[n++] = *s++;
    char d[8]; uint8_t m = 0;
    uint32_t v = g_area->size;
    do { d[m++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (m) line[n++] = d[--m];
    line[n++] = '\r'; line[n++] = '\n';
    return toWire((const uint8_t*)line, n);
}

static bool sendChosen(const char* buf, int len) {
    const uint8_t* b = (const uint8_t*)buf;
    uint16_t nb = blocks(len), off, n;
    g_baud  = BAUD[g_idx];
    g_runUp = false;

    if (g_area && Config::format == Export::DUMP && !sendSource()) return false;

    if (g_sel) {
        blockAt(g_sel, len, off, n);
        return Export::block(Config::format, b, off, n, g_sel, nb, toWire)
            && Export::finish(Config::format, b, (uint16_t)len, false, toWire);
    }
    for (uint16_t k = 1; k <= nb; k++) {
        blockAt(k, len, off, n);
        if (k > 1 && !pauseBetween()) return false;
        if (!Export::block(Config::format, b, off, n, k, nb, toWire)) return false;
    }
    return Export::finish(Config::format, b, (uint16_t)len, true, toWire);
}

bool poll(DSO152TFT& tft, const char* buf0, int len0) {
    pick(buf0, len0);
    const char* buf = g_buf;
    int         len = g_len;
    uint8_t was = g_idx;
    if (Keys::hit(Keys::RIGHT) && g_idx + 1 < NBAUD) g_idx++;
    if (Keys::hit(Keys::LEFT)  && g_idx > 0)         g_idx--;
    if (g_idx != was) drawBaud(tft);

    uint16_t nb = blocks(len), sel = g_sel;
    if (nb > 1) {
        if (Keys::hit(Keys::DOWN) && g_sel < nb) g_sel++;
        if (Keys::hit(Keys::UP)   && g_sel > 0)  g_sel--;
    }
    if (g_sel != sel) { drawChoice(tft, buf, len); drawReady(tft, len); }

    if (Keys::hit(Keys::RUN) && len > 0) {
        drawState(tft, "SENDING", -1, TH.trace);
        if (sendChosen(buf, len)) drawState(tft, "SENT", -1, TH.ok);
        else                      drawState(tft, "STOPPED", -1, TH.err);
        Keys::sync();                   // the RUN that stopped it is not a new start
    }

    if (Keys::hit(Keys::MODE) || Keys::press(Keys::OK) == Keys::LONG) return false;
    return true;
}

void leave() { CalOut::level(false); }

}
