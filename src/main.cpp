// DSO-152 as an RS232 protocol inspector on the analogue input.
//
// Nothing arrives over a USART here. The BNC goes through the scope's own
// front end into ADC1, TIM1 paces the conversions, DMA1 fills a circular
// buffer, and the byte stream is recovered from those samples. The front end
// takes RS232 swings directly, so no MAX3232 sits in front of it.
//
// Because the USART is gone, the two pins it had taken go back to being
// buttons: PA9 is DOWN again and PA10 is RUN again, which leaves PC4 as OK
// alone rather than doubling as both.
//
// The instrument starts on a menu (Menu.cpp) with two things to run: this
// inspector, and a sender (TtlApp.cpp) that puts what the inspector received
// out on the calibration output, as serial at logic levels. OK held brings
// either of them back to the menu. What was received stays in the buffer
// until it is erased from the sender. Memory (MemApp.cpp) shows the flash as
// it stands: the application, the settings page, the stock calibration. And
// Settings (SetApp.cpp) picks the theme and the way the sender exports, and
// keeps them in flash (Config.cpp). The signal generator of the projects this
// came from is not here.

#include <Arduino.h>
#include "dso152mod.h"
#include "Theme.h"
#include "DisplayHex.h"
#include "DisplayChars.h"
#include "DisplayLarge.h"
#include "Acquire.h"
#include "Frontend.h"
#include "Rs232.h"
#include "ProtMode.h"
#include "CalOut.h"
#include "Keys.h"
#include "Menu.h"
#include "TtlApp.h"
#include "SetApp.h"
#include "MemApp.h"
#include "Config.h"
#include "Export.h"

DSO152TFT tft;

#define TPRW          PC10

#define APP_MENU    0
#define APP_TESTER  1
#define APP_TTL     3
#define APP_SET     4
#define APP_MEM     5
static uint8_t app = APP_MENU;

#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240

#define MODE_PROT   0
#define MODE_HEX    1
#define MODE_CHARS  2
#define MODE_LARGE  3
#define NUM_MODES   4

#define BUFFER_SIZE 2048
static char dataBuffer[BUFFER_SIZE];
static int  bufferIndex = 0;
static int  lastBufferIndex = 0;

static uint8_t  displayMode = MODE_PROT;
static volatile bool running = true;
static int      lastDisplayedLines = 0;
static int      scrollFirstLine = -1;

// Pick the rate for the line speed: about ten samples a bit is enough to
// decode and leaves the most of the traffic on screen. 100 kS/s suits 9600,
// 250 k suits 19200 and 38400, 1.25 M suits 115200. Starting at the slowest
// because 9600 is what turns up most often.
static const uint32_t RATES[] = { 100000, 250000, 500000, 1250000, 2000000 };
static uint8_t rateIdx = 0;

// AUTO steps through these. The two fixed ones are 8N1 and bring their own
// sample rate, a little over ten samples a bit; the third works the speed and
// the shape out from the line, at whatever rate UP and DOWN have set.
#define SPEED_9600   0
#define SPEED_19200  1
#define SPEED_AUTO   2
static uint8_t speedMode = SPEED_9600;



// RUN steps through RUN, HOLD and SNAP. SNAP is a single shot: it waits for
// the first start bit, keeps the samples from just before it for as long as
// SNAP_CHARS characters take, and stops with them on the screen, so that a
// message that would be gone in a blink under RUN can be looked at and
// photographed. RUN pressed on a picture waits for the next one; pressed
// while still waiting it goes back to RUN.
//
// The picture holds SNAP_CHARS characters, but the message is received to
// its end: reading goes on after the picture is full, until the line has
// been at rest for SNAP_REST_MS or SNAP_MAX_CHARS have come. A scanned label
// longer than the picture is then all in the buffer, and its suffix can be
// looked at in the other views without scanning it again. Reading stops at
// the end of a block, so a few more than SNAP_MAX_CHARS may be taken.
#define SNAP_OFF    0
#define SNAP_ARMED  1       // nothing yet
#define SNAP_FILL   2       // the start bit has come, samples are being kept
#define SNAP_DONE   3       // received to the end, not painted yet
#define SNAP_SHOWN  4
#define SNAP_TAIL   5       // the picture is full, the rest of the message is being read
#define SNAP_CHARS      8
#define SNAP_MAX_CHARS  32
#define SNAP_REST_MS    50
static volatile uint8_t snap = SNAP_OFF;
static uint32_t snapBytes0 = 0;     // the count of characters when the message began
static uint32_t snapRest = 0;       // samples gone by with no start bit in them


// The waveform is drawn from a copy, not from the DMA buffer. A half lasts
// 410 us at 1.25 MS/s and the screen is only redrawn eight times a second,
// so by the time anything was painted the live buffer would have been
// written over several times and the trace would tear.
static uint32_t lastWaveMs = 0;
//
// SNAP keeps its samples here too. A message starts anywhere in a half, so
// its picture runs over into the next one, and the length is what eight
// characters need at the widest of the fixed speeds: 19200 at 250 kS/s is 13
// samples a bit, 1081 for eight frames with the lead in.
#define SNAP_LEN 1152
static uint16_t waveSnap[SNAP_LEN];
static uint16_t waveN = 0;

// Called with each block once it has been fed to the decoder, from wherever
// that happened. A block that went by unread while the picture was being
// filled leaves a gap, and the picture ends at what was kept before it.
static void snapBlock(const uint16_t* s, uint16_t n, bool over) {
    uint16_t from = 0;
    bool started = Rs232::firstStart() >= 0;

    if (snap == SNAP_ARMED) {
        if (!started) {
            snapBytes0 = Rs232::stats().bytes;      // nothing of the message yet
            return;
        }
        uint32_t pre = (Rs232::stats().bitSamples >> 8) * 2u;   // two bits of rest
        if (pre > 64) pre = 64;
        uint32_t at = (uint32_t)Rs232::firstStart();
        from  = at > pre ? (uint16_t)(at - pre) : 0;
        waveN = 0;
        snap  = SNAP_FILL;
    } else if (snap == SNAP_FILL) {
        if (over) snap = SNAP_TAIL;                 // a gap: the picture ends here
    } else if (snap != SNAP_TAIL) {
        return;
    }

    if (snap == SNAP_FILL) {
        uint16_t take = (uint16_t)(n - from);
        if (take > (uint16_t)(SNAP_LEN - waveN)) take = (uint16_t)(SNAP_LEN - waveN);
        memcpy(waveSnap + waveN, s + from, (size_t)take * sizeof(uint16_t));
        waveN = (uint16_t)(waveN + take);
        if (waveN >= SNAP_LEN) snap = SNAP_TAIL;
    }

    // The message is over when the line has rested, or is long enough.
    snapRest = started ? 0 : snapRest + n;
    bool rested = (uint64_t)snapRest * 1000u >= (uint64_t)Acq::sampleRate() * SNAP_REST_MS;
    bool enough = (Rs232::stats().bytes - snapBytes0) >= SNAP_MAX_CHARS;
    if (rested || enough) {
        running = false;
        snap    = SNAP_DONE;
    }
}

#ifndef DEMO_SIGNAL
// The decoder runs in the DMA interrupt, so no sample is missed while the
// screen is painted. The screen asks for a picture with wantSnap; the
// interrupt copies the half it has just decoded, together with the marks
// that go with it, and answers with snapReady. Nothing is asked for while a
// picture is being painted, so the copy is never written under the painter.
static volatile bool wantSnap = false, snapReady = false;

static void onBlock(const uint16_t* s, uint16_t n, bool over) {
    if (!running) return;
    if (over) Rs232::noteOverrun();
    Rs232::feed(s, n, Acq::sampleRate());
    if (snap != SNAP_OFF) { snapBlock(s, n, over); return; }
    if (wantSnap) {
        waveN = n > (uint16_t)(Acq::BUF_LEN / 2) ? (uint16_t)(Acq::BUF_LEN / 2) : n;
        memcpy(waveSnap, s, (size_t)waveN * sizeof(uint16_t));
        Rs232::latchView();
        wantSnap  = false;
        snapReady = true;
    }
}
#endif

#ifdef DEMO_SIGNAL
// A line that is not there, behaving like a hand scanner on a real adapter:
// a second of rest first, then a short message at 9600 8N1 with the
// characters back to back, then half a second of rest, and so on. The levels
// and the rest are copied from a real unit on range 7: mark at 1740 and space
// at 2440 counts, twenty counts of noise, and at rest a bump of sixty counts
// a quarter of a millisecond wide every so often, which is what was once
// taken for traffic. The bit time is the true 10.42 samples.
static uint16_t demoBuf[Acq::BUF_LEN / 2];
static const char DEMO_TEXT[] = "Apreciometru RS232\r";
static uint8_t  demoChar = 0, demoBit = 0;
static uint32_t demoAcc = 0, demoIdle = 100000;
static uint16_t demoNoise = 1, demoBump = 0;

static const uint16_t* demoBlock(uint16_t& n, uint32_t sampleHz) {
    const uint32_t baud = 9600;
    n = (uint16_t)(Acq::BUF_LEN / 2);

    for (uint16_t i = 0; i < n; i++) {
        // bit 0 is the start, 1..8 the data, 9 the stop
        bool mark = true;
        if (demoIdle) {
            demoIdle--;
            if ((demoIdle % 1700u) == 0) demoBump = 26;
        } else {
            uint8_t c = (uint8_t)DEMO_TEXT[demoChar];
            if (demoBit == 0)      mark = false;
            else if (demoBit <= 8) mark = ((c >> (demoBit - 1)) & 1) != 0;

            demoAcc += baud;
            if (demoAcc >= sampleHz) {
                demoAcc -= sampleHz;
                if (++demoBit >= 10) {
                    demoBit = 0;
                    if (DEMO_TEXT[++demoChar] == 0) {
                        demoChar = 0;
                        demoIdle = sampleHz / 2;
                    }
                }
            }
        }

        demoNoise = (uint16_t)(demoNoise * 1103u + 12345u);
        int16_t v = (int16_t)((mark ? 1740 : 2440) + (int16_t)((demoNoise >> 9) & 0x1F) - 16);
        if (demoBump) { demoBump--; v += 60; }
        demoBuf[i] = (uint16_t)v;
    }
    return demoBuf;
}
#endif

static void updateStatusBar() {
    Rs232::Stats st = Rs232::stats();
    tft.fillRect(0, 0, SCREEN_WIDTH, 20, TH.bar);
    tft.setTextSize(1);
    tft.setTextColor(TH.barText, TH.bar);

    tft.setCursor(2, 6);
    tft.print(snap == SNAP_OFF ? (running ? "RUN " : "HOLD")
                                    : (running ? "WAIT" : "SNAP"));

    tft.setCursor(44, 6);
    tft.print("R");
    tft.print(Frontend::range());
    tft.print(Frontend::couplingDC() ? " DC" : " AC");

    tft.setCursor(110, 6);
    if (speedMode == SPEED_AUTO) tft.print("A ");
    if (st.locked && st.baud) {
        Rs232::Format f = Rs232::format();
        tft.print(st.baud);
        tft.print(' ');
        tft.print(f.dataBits);
        tft.print(f.parity);
        tft.print(f.stopBits);
        if (!f.decided) tft.print('?');     // still striking candidates out
    } else {
        tft.print("baud ?");
    }

    tft.setCursor(200, 6);
    tft.print(Rs232::inverted() ? "RS232" : "TTL");

    tft.setCursor(240, 6);
    tft.print(st.bytes);
    tft.print(" B");
}

// The byte count moves all the time, the rest of the bar hardly ever, so the
// count is rewritten on its own and the bar only when something in it changed.
static void updateByteCount() {
    tft.setTextSize(1);
    tft.setTextColor(TH.barText, TH.bar);
    tft.setCursor(240, 6);
    tft.print(Rs232::stats().bytes);
    tft.print(" B  ");
}

static void refreshStatusBar() {
    static uint32_t lastMs = 0, lastKey = 0;
    uint32_t now = millis();
    if (now - lastMs < 500) return;
    lastMs = now;

    Rs232::Stats  st = Rs232::stats();
    Rs232::Format f  = Rs232::format();
    uint32_t key = (st.locked ? st.baud : 0)
                 ^ ((uint32_t)f.dataBits << 28) ^ ((uint32_t)f.parity << 20)
                 ^ ((uint32_t)f.stopBits << 26) ^ ((uint32_t)f.decided << 31);
    if (key != lastKey) { lastKey = key; updateStatusBar(); }
    else                updateByteCount();
}

static void paintProt() {
    Prot::drawWave(tft, waveSnap, waveN);
    Prot::drawBus(tft, waveN);
    Prot::drawLevels(tft, Acq::sampleRate(), dataBuffer, bufferIndex);
}

// The levels are held from one message to the next, so anything that moves
// them on the ADC -- another range, the coupling -- has to start them again.
static void relearn() {
    Acq::pause();
    Rs232::reset();
    Acq::resume();
}

static void applySpeedMode() {
    if (speedMode == SPEED_9600)  rateIdx = 0;
    if (speedMode == SPEED_19200) rateIdx = 1;
    Acq::setSampleRate(RATES[rateIdx]);
    Acq::pause();
    Rs232::setFixedBaud(speedMode == SPEED_9600 ? 9600 : speedMode == SPEED_19200 ? 19200 : 0);
    Rs232::reset();
    Acq::resume();
}

static void compactBuffer() {
    const int keep = BUFFER_SIZE / 2;
    memmove(dataBuffer, dataBuffer + bufferIndex - keep, keep);
    bufferIndex = keep;
    dataBuffer[bufferIndex] = 0;
    lastBufferIndex = 0;
    scrollFirstLine = -1;
}

static void drainDecoder() {
    int c;
    while ((c = Rs232::read()) >= 0) {
        if (bufferIndex >= BUFFER_SIZE - 1) compactBuffer();
        dataBuffer[bufferIndex++] = (char)c;
        dataBuffer[bufferIndex] = 0;
    }
}

static void updateTextMode(bool hex) {
    const int perLine  = hex ? HEX_PER_LINE  : CHARS_PER_LINE;
    const int visLines = hex ? HEX_VIS_LINES : CHARS_VIS_LINES;
    const int lineH    = hex ? HEX_LINE_H    : CHARS_LINE_H;

    int totalLines = (bufferIndex + perLine - 1) / perLine;
    int firstLine  = totalLines > visLines ? totalLines - visLines : 0;

    if (lastDisplayedLines == 0 || firstLine != scrollFirstLine) {
        tft.fillRect(0, 20, SCREEN_WIDTH, SCREEN_HEIGHT - 20, TH.bg);
        if (hex) displayHex(tft, dataBuffer, bufferIndex, firstLine);
        else     displayChars(tft, dataBuffer, bufferIndex, firstLine);
        scrollFirstLine = firstLine;
        lastDisplayedLines = 1;
        return;
    }
    int fromLine = lastBufferIndex / perLine;
    if (fromLine < firstLine) fromLine = firstLine;
    for (int ln = fromLine; ln < totalLines; ln++) {
        int y = 20 + (ln - firstLine) * lineH;
        if (hex) displayHexLine(tft, dataBuffer, bufferIndex, ln, y);
        else     displayCharsLine(tft, dataBuffer, bufferIndex, ln, y);
    }
}

static void redrawAll() {
    tft.fillScreen(TH.bg);
    updateStatusBar();
    lastDisplayedLines = 0;
    scrollFirstLine = -1;
    if (displayMode == MODE_PROT) {
        Prot::drawChrome(tft);
        if (snap == SNAP_SHOWN) paintProt();
    }
}

// Asked for from the sender. Whatever was decoded but not yet collected goes
// too, or it would turn up in the empty buffer the next time round.
static void eraseBuffer() {
    while (Rs232::read() >= 0) {}
    bufferIndex = 0;
    dataBuffer[0] = 0;
    lastBufferIndex = 0;
    lastDisplayedLines = 0;
    scrollFirstLine = -1;
}

static void armSnap() {
    Acq::pause();
#ifndef DEMO_SIGNAL
    wantSnap = false; snapReady = false;
#endif
    snap    = SNAP_ARMED;
    snapBytes0 = Rs232::stats().bytes;
    snapRest   = 0;
    running = true;
    Acq::resume();
    displayMode = MODE_PROT;            // the picture is drawn nowhere else
    redrawAll();
}

// The samples are all in. They are read once more from the top, this time
// for the marks and the characters that belong to exactly this stretch.
// A message that never came down the line: "Apreciometru RS232" and CR LF,
// made into samples the way the line would bring them, at the speed and the
// sample rate in force, and put through the decoder and the capture exactly
// as a block from the converter is. The waveform, the characters under it
// and the buffer then show it as if it had been received, which is a way to
// see every view working with nothing on the probe.
//
// The levels are those a real adapter gave on range 7, either side of zero
// volts; in TTL mode the other way up, since there the line rests high.
static void simulateMessage() {
    static const char MSG[] = "Apreciometru RS232\r\n";
    Rs232::Stats st = Rs232::stats();
    uint32_t hz   = Acq::sampleRate();
    uint32_t baud = Rs232::fixedBaud() ? Rs232::fixedBaud()
                  : (st.locked && st.baud) ? st.baud : 9600;
    bool     inv  = Rs232::inverted();
    uint16_t mark = inv ? 1716 : 2443, space = inv ? 2443 : 1716;

    Acq::pause();                       // no real block may come in between
    uint16_t blk[256];
    uint16_t ch = 0, bit = 0, noise = 1;
    uint32_t acc = 0, lead = 30;        // some rest first, for the start bit to stand out
    bool     sending = true;

    // Blocks until the capture says the message is over, which it does once
    // the line has rested after it. The guard is far beyond what that takes.
    for (uint16_t guard = 0; snap != SNAP_DONE && guard < 600; guard++) {
        IWDG->KR = 0xAAAA;
        for (uint16_t i = 0; i < 256; i++) {
            bool m = true;
            if (lead) {
                lead--;
            } else if (sending) {
                uint8_t c = (uint8_t)MSG[ch];
                if (bit == 0)      m = false;                   // start
                else if (bit <= 8) m = ((c >> (bit - 1)) & 1) != 0;
                acc += baud;
                if (acc >= hz) {
                    acc -= hz;
                    if (++bit >= 10) { bit = 0; if (MSG[++ch] == 0) sending = false; }
                }
            }
            noise = (uint16_t)(noise * 1103u + 12345u);
            blk[i] = (uint16_t)((m ? mark : space) + ((noise >> 9) & 15) - 8);
        }
        Rs232::feed(blk, 256, hz);
        snapBlock(blk, 256, false);
    }

    Acq::discard();                     // what the converter did meanwhile is not wanted
    Acq::resume();
}

static void showSnap() {
    drainDecoder();
    waveN = Rs232::review(waveSnap, waveN, SNAP_CHARS);
    snap  = SNAP_SHOWN;
    if (displayMode == MODE_PROT) paintProt();
    updateStatusBar();
}

static void updateDisplay() {
    if (displayMode == MODE_PROT) {
        if (bufferIndex != lastBufferIndex || lastDisplayedLines == 0) {
            Prot::drawStream(tft, dataBuffer, bufferIndex);
            lastBufferIndex = bufferIndex;
            lastDisplayedLines = 1;
        }
        return;
    }

    if (bufferIndex != lastBufferIndex || lastDisplayedLines == 0) {
        switch (displayMode) {
            case MODE_HEX:   updateTextMode(true);  break;
            case MODE_CHARS: updateTextMode(false); break;
            case MODE_LARGE: {
                int startPos = (bufferIndex >= 32) ? bufferIndex - 32 : 0;
                if (largeBufferChanged(tft, dataBuffer, bufferIndex, startPos)
                    || lastDisplayedLines == 0) {
                    displayLarge(tft, dataBuffer, bufferIndex, startPos);
                    lastDisplayedLines = 1;
                }
                break;
            }
        }
        lastBufferIndex = bufferIndex;
    }
}

// Coming in from the menu. Whatever button is still down from the choosing
// is taken as seen, and the half that settled while nothing was collecting
// is let go rather than counted as lost.
static void testerEnter() {
    Keys::sync();
    redrawAll();
    Acq::discard();
    Acq::resume();
}

// Back to the menu. What has been received and learnt is kept for next time.
static void testerLeave() { Acq::pause(); }

static void showMenu() {
    app = APP_MENU;
    Keys::sync();
    Menu::draw(tft);
}

#ifdef EXPORT_SELFTEST
// With no receiver on the wire there is no seeing what the sender puts out.
// This build exports a known buffer into memory at start, as a hex dump and
// then as Intel HEX, in blocks of 64, for tools/selftest_check.py to read
// back over SWD and put through tools/ttl_receive.py.
uint8_t  selfOut[1400];
uint16_t selfLen[2];
static uint16_t selfN;
static bool selfSink(const uint8_t* p, uint16_t n) {
    if (selfN + n > sizeof(selfOut)) return false;
    memcpy(selfOut + selfN, p, n);
    selfN = (uint16_t)(selfN + n);
    return true;
}
static void exportSelfTest() {
    static uint8_t src[150];
    for (uint16_t i = 0; i < sizeof(src); i++) src[i] = (uint8_t)(i * 37u + 11u);
    for (uint8_t f = 0; f < 2; f++) {
        uint8_t fmt = f ? Export::IHEX : Export::DUMP;
        uint16_t from = selfN;
        for (uint16_t k = 0; k < 3; k++) {
            uint16_t off = (uint16_t)(k * 64), n = (uint16_t)(sizeof(src) - off < 64 ? sizeof(src) - off : 64);
            Export::block(fmt, src, off, n, (uint16_t)(k + 1), 3, selfSink);
        }
        Export::finish(fmt, src, sizeof(src), true, selfSink);
        selfLen[f] = (uint16_t)(selfN - from);
    }
}
#endif

void setup() {
    Keys::begin();

    pinMode(TPRW, OUTPUT);
    digitalWrite(TPRW, HIGH);
    delay(300);

    Config::load();                     // the theme, before anything is drawn
    tft.begin();
#ifdef EXPORT_SELFTEST
    exportSelfTest();
#endif
    dataBuffer[0] = 0;

    Frontend::begin();
    CalOut::begin();
    Rs232::begin();
#ifndef DEMO_SIGNAL
    Acq::onBlock(onBlock);
#endif
    Acq::begin(RATES[rateIdx]);
    Acq::pause();                       // nothing is read until the inspector is started

    showMenu();
}

static bool testerLoop();

void loop() {
    IWDG->KR = 0xAAAA;                  // started in ClockHandover.cpp

    if (digitalRead(POWER_BUTTON) == LOW) {
        digitalWrite(TPRW, LOW);
        delay(200);
    }

    switch (app) {
        case APP_MENU: {
            int8_t pick = Menu::poll(tft);
            if (pick == Menu::ITEM_TESTER)   { app = APP_TESTER; testerEnter(); }
            else if (pick == Menu::ITEM_SET) { app = APP_SET;    Settings::enter(tft, bufferIndex); }
            else if (pick == Menu::ITEM_MEM) { app = APP_MEM;    Memory::enter(tft); }
            else if (pick == Menu::ITEM_TTL) {
                // it starts at the speed the data came in at
                Rs232::Stats st = Rs232::stats();
                app = APP_TTL;
                Ttl::enter(tft, dataBuffer, bufferIndex, (st.locked && st.baud) ? st.baud : 9600);
            }
            break;
        }
        case APP_TTL: {
            if (!Ttl::poll(tft, dataBuffer, bufferIndex)) { Ttl::leave(); showMenu(); }
            break;
        }
        case APP_SET: {
            uint8_t r = Settings::poll(tft);
            if (r == Settings::ERASE)      { eraseBuffer(); Settings::erased(tft); }
            else if (r == Settings::LEAVE) showMenu();
            break;
        }
        case APP_MEM:
            if (!Memory::poll(tft)) showMenu();
            break;
        case APP_TESTER:
            if (!testerLoop()) { testerLeave(); showMenu(); }
            break;
    }
}

// One turn of the inspector. False: OK was held, back to the menu.
static bool testerLoop() {

#ifndef DEMO_SIGNAL
    if (running && Acq::interruptDriven()) {
        drainDecoder();
        if (snapReady) {
            snapReady = false;
            if (displayMode == MODE_PROT) paintProt();
            lastWaveMs = millis();
        } else if (!wantSnap && snap == SNAP_OFF && displayMode == MODE_PROT
                   && (millis() - lastWaveMs) >= 120) {
            wantSnap = true;
        }
    } else
#endif
    if (running) {
        uint16_t n;
        bool over = false;
#ifdef DEMO_SIGNAL
        const uint16_t* half = demoBlock(n, Acq::sampleRate());
        delay(2);                       // roughly the pace of a real block
#else
        const uint16_t* half = Acq::readyHalf(n, over);
#endif
        if (half) {
            if (over) Rs232::noteOverrun();
            Rs232::feed(half, n, Acq::sampleRate());
            drainDecoder();

            // Painted here rather than later: the bit marks and the byte
            // positions belong to the feed just done, and the next one would
            // replace them.
            uint32_t now = millis();
            if (snap != SNAP_OFF) {
                snapBlock(half, n, over);
            } else if (displayMode == MODE_PROT && (now - lastWaveMs) >= 120) {
                lastWaveMs = now;
                waveN = n > (uint16_t)(Acq::BUF_LEN / 2) ? (uint16_t)(Acq::BUF_LEN / 2) : n;
                memcpy(waveSnap, half, (size_t)waveN * sizeof(uint16_t));
                Rs232::latchView();
                paintProt();
            }
        }
    }

    if (snap == SNAP_DONE) showSnap();

    // MODE changes the view; held while a capture is awaited, it sends the
    // made-up message below instead of waiting for a real one.
    uint8_t mode = Keys::press(Keys::MODE);
    if (mode == Keys::SHORT) {
        displayMode = (uint8_t)((displayMode + 1) % NUM_MODES);
        redrawAll();
    } else if (mode == Keys::LONG && snap == SNAP_ARMED) {
        simulateMessage();
    }

    if (Keys::hit(Keys::RUN)) {
        if (snap == SNAP_OFF && running) {
            running = false;                        // RUN -> HOLD
            updateStatusBar();
        } else if (snap == SNAP_OFF || snap == SNAP_SHOWN) {
            armSnap();                              // HOLD -> SNAP, or once more
        } else {
            Acq::pause();                           // still waiting -> RUN
            snap    = SNAP_OFF;
            running = true;
            Acq::resume();
            updateStatusBar();
        }
    }

    if (Keys::hit(Keys::RIGHT)) {
        if (Frontend::range() < Frontend::RANGE_MAX) {
            Frontend::setRange((uint8_t)(Frontend::range() + 1));
            relearn();
        }
        updateStatusBar();
    }

    if (Keys::hit(Keys::LEFT)) {
        if (Frontend::range() > 0) {
            Frontend::setRange((uint8_t)(Frontend::range() - 1));
            relearn();
        }
        updateStatusBar();
    }

    if (Keys::hit(Keys::UP)) {
        if (rateIdx + 1 < (uint8_t)(sizeof(RATES) / sizeof(RATES[0]))) {
            rateIdx++;
            Acq::setSampleRate(RATES[rateIdx]);
            Acq::pause();
            Rs232::reset();
            Acq::resume();
        }
        updateStatusBar();
    }

    if (Keys::hit(Keys::DOWN)) {
        if (rateIdx > 0) {
            rateIdx--;
            Acq::setSampleRate(RATES[rateIdx]);
            Acq::pause();
            Rs232::reset();
            Acq::resume();
        }
        updateStatusBar();
    }

    if (Keys::hit(Keys::AUTO)) {
        speedMode = (uint8_t)((speedMode + 1) % 3);
        applySpeedMode();
        updateStatusBar();
    }

    // OK held goes back to the menu, so a short press is acted on when the
    // button comes up, not when it goes down.
    uint8_t ok = Keys::press(Keys::OK);
    if (ok == Keys::LONG) return false;
    if (ok == Keys::SHORT) {
        Acq::pause();
        Rs232::setInverted(!Rs232::inverted());   // RS232 swing, or plain TTL
        Rs232::reset();
        Acq::resume();
        updateStatusBar();
    }

    if (Keys::hit(Keys::RELEARN)) {
        Acq::pause();
        Rs232::reset();                     // measure the speed and shape again
        Acq::resume();
        updateStatusBar();
    }

    // Nothing is painted while a picture is awaited or being filled: above
    // IRQ_MAX_HZ the halves are fetched by this loop, and one that went by
    // during a repaint would cut the picture short.
    if (snap == SNAP_ARMED || snap == SNAP_FILL || snap == SNAP_TAIL) return true;

    updateDisplay();
    refreshStatusBar();
    return true;
}
