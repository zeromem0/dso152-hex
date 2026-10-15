#include <Arduino.h>
#include "Rs232.h"

namespace Rs232 {

// --- level discrimination -------------------------------------------------
static uint16_t g_lo = 4095, g_hi = 0;
static uint16_t g_blkLo = 0, g_blkHi = 0;   // of the last block, for the screen
static bool     g_haveLevels = false;

// A block has to swing at least this far to count as traffic. The front end
// idles with about a dozen counts of noise; an RS232 line on the least
// sensitive range still gives about a hundred.
static const uint16_t MIN_SPAN = 48;

// A span alone does not make traffic. With the cable on and the line at
// rest, a real adapter put bumps of some sixty counts on range 7, a quarter
// of a millisecond wide, all of them on the mark side of zero; taken for
// bits they gave 3846 baud and a stream of FF. Two more things are asked of
// a block before it is believed:
//
//   Both levels are held for a while, PLATEAU samples running in the upper
//   third and as many in the lower. A spike does not do that.
//
//   For an RS232 line, the swing goes through zero volts. Mark is negative
//   and space positive, so a block that stays on one side is a line at rest
//   whatever rides on it. Zero volts sat at 2078 counts on the unit measured,
//   on range 10 and on range 7 alike; ZERO_MARGIN allows for another unit
//   being a little off that.
static const uint16_t PLATEAU     = 6;
static const uint16_t ADC_ZERO    = 2078;
static const uint16_t ZERO_MARGIN = 16;
static uint16_t g_th = 2048, g_hyst = 16;
static bool     g_inverted = true;
static bool     g_one = true;               // current logical level
static bool     g_rawHigh = false;          // the level before the RS232 sense

// --- bit time -------------------------------------------------------------
static uint16_t g_shortest[8];
static uint16_t g_runs;
static uint32_t g_run;
static uint32_t g_bitQ8;
static uint32_t g_baud;
static bool     g_locked;

// Until traffic has been measured the line is taken to be 9600 baud, which
// is what it nearly always is. A first message too short to measure, three
// characters say, is then still read. `g_assumed` marks a bit time that is
// this guess rather than a measurement, so that the first real estimate is
// allowed to replace it.
static const uint32_t DEFAULT_BAUD = 9600;
static bool     g_assumed;

// Not zero: this speed, 8N1, and none of the learning below takes part.
static uint32_t g_fixedBaud = 9600;

// --- frame shape ----------------------------------------------------------
// Every candidate, in the order they are preferred when more than one still
// fits. Most constrained first, which is the opposite of most common, and
// deliberately so: 8N1 is never contradicted by 7E1 traffic, because it
// simply reads the parity bit as the top data bit and finds the stop where
// it expects it. 7E1 put first, a wrong parity guess dies within a few
// characters -- surviving twelve parity checks by luck is one chance in
// four thousand -- while a wrong 8N1 guess would never die at all.
// Within one data and parity pair the shorter frame comes first, since an
// idle line looks exactly like a second stop bit.
struct Cand { uint8_t data; uint8_t par; uint8_t stop; };   // par 0 N, 1 E, 2 O
static const Cand CAND[] = {
    { 7, 1, 1 }, { 7, 2, 1 }, { 8, 1, 1 }, { 8, 2, 1 }, { 7, 0, 1 }, { 8, 0, 1 },
    { 7, 1, 2 }, { 7, 2, 2 }, { 8, 1, 2 }, { 8, 2, 2 }, { 7, 0, 2 }, { 8, 0, 2 }
};
static const uint8_t NCAND = sizeof(CAND) / sizeof(CAND[0]);
static const uint8_t LEARN_BITS = 12;       // captured after the start bit
static const uint8_t LEARN_FRAMES = 12;     // before the answer is trusted

static uint16_t g_alive;                    // one bit a candidate
static uint8_t  g_chkN, g_chkBad;           // frames since the last check, and bad ones
static uint8_t  g_seen;
static bool     g_decided;
static uint8_t  g_fmtIdx;

// --- frame assembly -------------------------------------------------------
static bool     g_frame;
static int32_t  g_phaseQ8;
static uint8_t  g_fn;                       // bits captured after the start
static uint16_t g_fbits;                    // bit i of the frame in position i
static uint8_t  g_wantBits;                 // how many to take once decided
static int32_t  g_frameStart;
static int32_t  g_firstStart = -1;          // first start bit of the last feed

// --- output ---------------------------------------------------------------
// Filled by feed(), which may run in an interrupt, and emptied by the main
// loop once a turn. A turn that repaints the screen takes a good part of a
// second, and at 9600 baud a file being sent delivers 960 characters in
// that second: 256 places lost one character in five. 2048 is two seconds.
static const uint16_t FIFO_MASK = 2047;
static uint8_t  g_fifo[FIFO_MASK + 1];
static volatile uint16_t g_head, g_tail;
static uint32_t g_bytes, g_ferr, g_perr, g_over, g_drop;

// --- what the screen draws ------------------------------------------------
static uint16_t g_marks[MARKS_MAX];
static uint8_t  g_markN;
static uint16_t g_byteAt[BYTES_MAX];
static uint8_t  g_byteVal[BYTES_MAX];
static uint8_t  g_byteN;

static uint16_t v_marks[MARKS_MAX];
static uint8_t  v_markN;
static uint16_t v_byteAt[BYTES_MAX];
static uint8_t  v_byteVal[BYTES_MAX];
static uint8_t  v_byteN;

static const uint32_t STD_BAUD[] = {
    1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400
};

static inline uint8_t candBits(const Cand& c) {
    return (uint8_t)(c.data + (c.par ? 1 : 0) + c.stop);
}

static void pushByte(uint8_t b) {
    uint16_t next = (uint16_t)((g_head + 1) & FIFO_MASK);
    if (next == g_tail) { g_drop++; return; }
    g_fifo[g_head] = b;
    g_head = next;
    g_bytes++;
}

void begin() { reset(); }

static void relearnFormat() {
    g_alive = (uint16_t)((1u << NCAND) - 1u);
    g_seen = 0; g_decided = false; g_fmtIdx = 5;   // 8N1 until shown otherwise
    g_wantBits = 9;      // 8N1 and 7E1 both, the usual two
    g_chkN = 0; g_chkBad = 0;
}

// Forget the speed and the shape but not the levels.
static void relearn() {
    for (uint8_t i = 0; i < 8; i++) g_shortest[i] = 0xFFFF;
    g_runs = 0; g_run = 0;
    g_bitQ8 = 0; g_baud = 0; g_locked = false; g_assumed = false;
    relearnFormat();
    g_frame = false; g_fn = 0; g_fbits = 0; g_frameStart = -1;

    if (g_fixedBaud) {
        g_fmtIdx   = 5;                     // 8N1
        g_alive    = (uint16_t)(1u << 5);
        g_seen     = LEARN_FRAMES;
        g_decided  = true;
        g_wantBits = candBits(CAND[5]);
        g_locked   = true;
        g_baud     = g_fixedBaud;           // the bit time follows in feed()
    }
}

void     setFixedBaud(uint32_t baud) { g_fixedBaud = baud; }
uint32_t fixedBaud() { return g_fixedBaud; }

void reset() {
    relearn();
    g_markN = 0; g_byteN = 0;
    v_markN = 0; v_byteN = 0;
    g_lo = 4095; g_hi = 0;
    g_haveLevels = false;
}

void setInverted(bool inv) { g_inverted = inv; }
bool inverted() { return g_inverted; }
void noteOverrun() { g_over++; }

int available() { return (int)((g_head - g_tail) & FIFO_MASK); }

int read() {
    if (g_head == g_tail) return -1;
    uint8_t b = g_fifo[g_tail];
    g_tail = (uint16_t)((g_tail + 1) & FIFO_MASK);
    return b;
}

static void offerRun(uint32_t len) {
    if (len == 0 || len > 60000u) return;
    uint16_t v = (uint16_t)len;
    for (uint8_t i = 0; i < 8; i++) {
        if (v < g_shortest[i]) { uint16_t t = g_shortest[i]; g_shortest[i] = v; v = t; }
    }
    g_runs++;

    // Once the bit time has been pulled onto a standard speed it is a
    // fraction of a sample better than any whole run length, so a new
    // estimate replaces it only if the two really disagree.
    if (g_runs >= 24 && g_shortest[4] != 0xFFFF) {
        uint32_t bit = g_shortest[4];
        if (bit >= 2) {
            uint32_t q = bit << 8;
            uint32_t d = q > g_bitQ8 ? q - g_bitQ8 : g_bitQ8 - q;
            if (!g_locked || d * 8u > g_bitQ8) g_bitQ8 = q;
            g_locked  = true;
            g_assumed = false;
        }
    }

    // Start the measurement again every so often. The set only ever shrinks,
    // so a single glitch early on would otherwise hold the estimate too short
    // for as long as the instrument stayed on.
    if (g_runs >= 512) {
        for (uint8_t i = 0; i < 8; i++) g_shortest[i] = 0xFFFF;
        g_runs = 0;
    }
}

// A run is a whole number of samples, so at ten samples a bit the raw figure
// is out by up to one part in ten either way. Eight per cent still keeps the
// standard speeds well apart: the closest two are half as far again.
static bool nearStandard(uint32_t raw) {
    for (uint8_t i = 0; i < sizeof(STD_BAUD) / sizeof(STD_BAUD[0]); i++) {
        uint32_t s = STD_BAUD[i];
        uint32_t d = raw > s ? raw - s : s - raw;
        if (d * 25u <= s * 2u) return true;
    }
    return false;
}

static void snapBaud(uint32_t sampleHz) {
    if (!g_locked || g_bitQ8 == 0) { g_baud = 0; return; }
    uint32_t raw = (uint32_t)(((uint64_t)sampleHz << 8) / g_bitQ8);
    g_baud = raw;
    for (uint8_t i = 0; i < sizeof(STD_BAUD) / sizeof(STD_BAUD[0]); i++) {
        uint32_t s = STD_BAUD[i];
        uint32_t d = raw > s ? raw - s : s - raw;
        if (d * 25u <= s * 2u) {
            g_baud  = s;
            g_bitQ8 = (uint32_t)(((uint64_t)sampleHz << 8) / s);
            break;
        }
    }
}

// Extract the data of one frame under a given candidate, or return -1 if the
// frame contradicts it.
static int16_t tryCand(const Cand& c, uint16_t bits, uint8_t have) {
    uint8_t need = candBits(c);
    if (have < need) return -1;

    uint8_t data = 0, ones = 0;
    for (uint8_t i = 0; i < c.data; i++) {
        if (bits & (1u << i)) { data |= (uint8_t)(1u << i); ones++; }
    }
    uint8_t at = c.data;
    if (c.par) {
        if (bits & (1u << at)) ones++;
        at++;
        // even parity wants an even count over data and parity together
        if (c.par == 1 && (ones & 1)) return -1;
        if (c.par == 2 && !(ones & 1)) return -1;
    }
    for (uint8_t i = 0; i < c.stop; i++) {
        if (!(bits & (1u << (at + i)))) return -1;    // a stop bit must be mark
    }
    return (int16_t)data;
}

static void finishFrame() {
    if (!g_decided) {
        // strike out every candidate this character contradicts
        // A candidate longer than what was captured is neither confirmed nor
        // struck out: there is nothing to judge it on yet.
        uint16_t still = 0;
        for (uint8_t i = 0; i < NCAND; i++) {
            if (!(g_alive & (1u << i))) continue;
            if (candBits(CAND[i]) > g_fn) { still |= (uint16_t)(1u << i); continue; }
            if (tryCand(CAND[i], g_fbits, g_fn) >= 0) still |= (uint16_t)(1u << i);
        }
        if (still) g_alive = still;          // never strike them all out
        if (g_seen < 255) g_seen++;

        // Two different questions, and answering both with one candidate is
        // what went wrong twice over. The window must follow the SHORTEST
        // candidate still standing, because a window longer than the frame
        // eats the next character's start bit and every frame after it is
        // read off its boundary. What gets reported must be the most
        // preferred candidate that has actually been judged: one too long
        // for the window has never been tested at all, and reporting it
        // would be reporting a guess nothing has yet contradicted.
        uint8_t shortest = 0xFF;
        uint8_t best = 0xFF;
        for (uint8_t i = 0; i < NCAND; i++) {
            if (!(g_alive & (1u << i))) continue;
            uint8_t cb = candBits(CAND[i]);
            if (cb < shortest) shortest = cb;
            if (best == 0xFF && cb <= g_fn) best = i;
        }
        if (best != 0xFF) g_fmtIdx = best;
        int16_t v = (candBits(CAND[g_fmtIdx]) <= g_fn)
                  ? tryCand(CAND[g_fmtIdx], g_fbits, g_fn) : -1;
        if (v >= 0) {
            pushByte((uint8_t)v);
            if (g_frameStart >= 0 && g_byteN < BYTES_MAX) {
                g_byteAt[g_byteN]  = (uint16_t)g_frameStart;
                g_byteVal[g_byteN] = (uint8_t)v;
                g_byteN++;
            }
        } else {
            g_ferr++;
        }

        // Follow the current best guess straight away rather than waiting
        // for the answer. On back to back traffic a window longer than the
        // frame runs into the next character and every frame after it is
        // read off its proper boundary, which strikes out the right
        // candidate and leaves the loosest one standing. If the length is
        // wrong the stop bits stop landing on mark, that candidate dies, and
        // the next one brings its own length with it.
        g_wantBits = (shortest == 0xFF) ? 9 : shortest;
        if (g_seen >= LEARN_FRAMES && best != 0xFF) g_decided = true;
    } else {
        int16_t v = tryCand(CAND[g_fmtIdx], g_fbits, g_fn);
        if (v >= 0) {
            pushByte((uint8_t)v);
            if (g_frameStart >= 0 && g_byteN < BYTES_MAX) {
                g_byteAt[g_byteN]  = (uint16_t)g_frameStart;
                g_byteVal[g_byteN] = (uint8_t)v;
                g_byteN++;
            }
        } else {
            // tell a bad stop bit from a bad parity, they mean different things
            const Cand& c = CAND[g_fmtIdx];
            uint8_t at = (uint8_t)(c.data + (c.par ? 1 : 0));
            bool stopOk = true;
            for (uint8_t i = 0; i < c.stop; i++)
                if (!(g_fbits & (1u << (at + i)))) stopOk = false;
            if (stopOk) g_perr++; else g_ferr++;
            g_chkBad++;
        }
        // A shape can be settled on wrongly, from a dozen characters that
        // were not characters. Nothing used to undo that. Now a settled
        // shape that goes on failing half the time is thrown out and worked
        // out again from what is actually arriving.
        if (++g_chkN >= 16) {
            if (g_chkBad >= 8 && !g_fixedBaud) relearnFormat();
            g_chkN = 0; g_chkBad = 0;
        }
    }
    g_frame = false;
    g_frameStart = -1;
}

static bool looksLikeTraffic(const uint16_t* s, uint16_t n, uint16_t lo, uint16_t hi) {
    if (hi < lo) return false;
    uint16_t span = (uint16_t)(hi - lo);
    if (span < MIN_SPAN) return false;
    if (g_inverted
        && !((uint16_t)(lo + ZERO_MARGIN) < ADC_ZERO && hi > (uint16_t)(ADC_ZERO + ZERO_MARGIN)))
        return false;

    uint16_t upper = (uint16_t)(hi - span / 3u);
    uint16_t lower = (uint16_t)(lo + span / 3u);
    uint16_t runHi = 0, runLo = 0, bestHi = 0, bestLo = 0;
    for (uint16_t i = 0; i < n; i++) {
        uint16_t v = s[i] & 0x0FFF;
        if (v >= upper)      { runLo = 0; if (++runHi > bestHi) bestHi = runHi; }
        else if (v <= lower) { runHi = 0; if (++runLo > bestLo) bestLo = runLo; }
        else                 { runHi = 0; runLo = 0; }
    }
    return bestHi >= PLATEAU && bestLo >= PLATEAU;
}

// One pass over a block. `learn` offers the run lengths to the speed
// estimate, `decode` lets frames start. They are apart because the block in
// which the speed is first found is gone over twice: once to find it, and
// again from the top to read the characters that went by before it was known.
static void scan(const uint16_t* s, uint16_t n, bool learn, bool decode) {
    for (uint16_t i = 0; i < n; i++) {
        uint16_t v = s[i] & 0x0FFF;

        if (v > (uint16_t)(g_th + g_hyst))      g_rawHigh = true;
        else if (v < (uint16_t)(g_th - g_hyst)) g_rawHigh = false;
        bool one = g_inverted ? !g_rawHigh : g_rawHigh;

        if (one != g_one) {
            if (learn) offerRun(g_run);
            g_run = 0;
            if (decode && !g_frame && g_one && !one && g_locked) {
                g_frame      = true;
                g_frameStart = (int32_t)i;
                if (g_firstStart < 0) g_firstStart = (int32_t)i;
                g_phaseQ8    = (int32_t)(g_bitQ8 + (g_bitQ8 >> 1));
                g_fn         = 0;
                g_fbits      = 0;
            }
            g_one = one;
        }
        g_run++;

        if (decode && g_frame) {
            g_phaseQ8 -= 256;
            if (g_phaseQ8 <= 0) {
                if (g_markN < MARKS_MAX) g_marks[g_markN++] = i;
                if (one) g_fbits |= (uint16_t)(1u << g_fn);
                g_fn++;
                g_phaseQ8 += (int32_t)g_bitQ8;

                if (g_fn >= g_wantBits) finishFrame();
            }
        }
    }
}

// A short message, a scanned barcode say, is over before two dozen runs have
// gone by. So the speed is also taken early, from the second shortest run of
// as few as eight, but only if that lands on a standard speed; the fuller
// estimate in offerRun() takes over once there is enough to go on.
static void earlyLock(uint32_t sampleHz) {
    if ((g_locked && !g_assumed) || g_runs < 8) return;
    uint16_t bit = g_shortest[1];
    if (bit == 0xFFFF || bit < 2) return;
    if (!nearStandard(sampleHz / bit)) return;
    uint32_t q = (uint32_t)bit << 8;
    uint32_t d = q > g_bitQ8 ? q - g_bitQ8 : g_bitQ8 - q;
    if (!g_locked || d * 8u > g_bitQ8) g_bitQ8 = q;    // else the guess was right
    g_locked  = true;
    g_assumed = false;
}

void feed(const uint16_t* s, uint16_t n, uint32_t sampleHz) {
    if (!s || n == 0) return;

    uint16_t lo = 4095, hi = 0;
    for (uint16_t i = 0; i < n; i++) {
        uint16_t v = s[i] & 0x0FFF;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    g_blkLo = lo;
    g_blkHi = hi;

    // The two levels are an envelope that is held, not an average of what
    // each block happens to contain. A line spends most of its time at rest,
    // and an average over blocks of nothing but rest closes in on the noise
    // around the mark level: the threshold then sits inside that noise, every
    // wiggle reads as an edge, and the speed and the frames are made of them.
    // So a block only moves the levels if it has a real swing in it; it moves
    // them outward at once, so that a message is split correctly from its
    // first block, and inward slowly.
    if (looksLikeTraffic(s, n, lo, hi)) {
        if (!g_haveLevels) { g_lo = lo; g_hi = hi; g_haveLevels = true; }
        else if (!g_fixedBaud && (uint32_t)(hi - lo) > 2u * (uint32_t)(g_hi - g_lo)) {
            // A swing more than twice what was being followed is another
            // signal, not more of the same; whatever was learnt from the
            // smaller one is not about this.
            relearn();
            g_lo = lo; g_hi = hi;
        }
        else {
            g_lo = lo < g_lo ? lo : (uint16_t)((7u * g_lo + lo + 7u) >> 3);
            g_hi = hi > g_hi ? hi : (uint16_t)((7u * g_hi + hi) >> 3);
        }
    }

    g_markN = 0;
    g_byteN = 0;
    g_frameStart = -1;                      // a frame under way began earlier
    g_firstStart = -1;

    if (!g_haveLevels) return;              // nothing to split on yet

    uint16_t span = (g_hi > g_lo) ? (uint16_t)(g_hi - g_lo) : 0;
    g_th   = (uint16_t)((g_lo + g_hi) >> 1);
    g_hyst = (uint16_t)(span >> 3);
    if (g_hyst < 8) g_hyst = 8;

    if (g_fixedBaud) {
        g_bitQ8  = (uint32_t)(((uint64_t)sampleHz << 8) / g_fixedBaud);
        g_baud   = g_fixedBaud;
        g_locked = true;
        scan(s, n, false, true);
        return;
    }

    // With the speed measured, one pass does both jobs. Until then the block
    // is gone over twice: first only to measure, then, with the measurement
    // or failing that the 9600 guess, from the top again to read it. The
    // first pass leaves a frame carried over from the block before alone.
    bool measured = g_locked && !g_assumed;
    if (measured) {
        scan(s, n, true, true);
        snapBaud(sampleHz);
        return;
    }

    bool     one0 = g_one, raw0 = g_rawHigh;
    uint32_t run0 = g_run;

    scan(s, n, true, false);
    earlyLock(sampleHz);
    if (!g_locked) {
        g_bitQ8   = (uint32_t)(((uint64_t)sampleHz << 8) / DEFAULT_BAUD);
        g_locked  = true;
        g_assumed = true;
    }
    snapBaud(sampleHz);

    g_one = one0; g_rawHigh = raw0; g_run = run0;
    scan(s, n, false, true);
}

Stats stats() {
    Stats st;
    if (g_haveLevels) { st.lo = g_lo; st.hi = g_hi; st.threshold = g_th; }
    else {
        st.lo = g_blkLo; st.hi = g_blkHi;
        st.threshold = (uint16_t)((g_blkLo + g_blkHi) >> 1);
    }
    st.baud = g_baud; st.bitSamples = g_bitQ8;
    st.bytes = g_bytes; st.framingErrors = g_ferr;
    st.parityErrors = g_perr; st.overruns = g_over; st.dropped = g_drop;
    st.locked = g_locked;
    st.levels = g_haveLevels;
    return st;
}

Format format() {
    Format f;
    const Cand& c = CAND[g_fmtIdx];
    f.dataBits   = c.data;
    f.parity     = c.par == 0 ? 'N' : (c.par == 1 ? 'E' : 'O');
    f.stopBits   = c.stop;
    f.decided    = g_decided;
    f.framesSeen = g_seen;
    return f;
}

void latchView() {
    v_markN = g_markN;
    memcpy(v_marks, g_marks, (size_t)g_markN * sizeof(g_marks[0]));
    v_byteN = g_byteN;
    memcpy(v_byteAt,  g_byteAt,  (size_t)g_byteN * sizeof(g_byteAt[0]));
    memcpy(v_byteVal, g_byteVal, (size_t)g_byteN);
}

int32_t firstStart() { return g_firstStart; }

uint16_t review(const uint16_t* s, uint16_t n, uint8_t maxBytes) {
    v_markN = 0; v_byteN = 0;
    if (!s || n == 0 || !g_haveLevels || !g_locked || g_bitQ8 == 0) return n;

    // The characters in here have been through feed() once already, so the
    // queue and the counts are put back as they were found.
    uint16_t head = g_head;
    uint32_t bytes = g_bytes, ferr = g_ferr, perr = g_perr, drop = g_drop;

    g_frame = false; g_fn = 0; g_fbits = 0; g_frameStart = -1;
    g_one = true; g_rawHigh = !g_inverted; g_run = 0;
    g_markN = 0; g_byteN = 0;
    scan(s, n, false, true);
    g_frame = false; g_frameStart = -1;
    g_one = true; g_rawHigh = !g_inverted; g_run = 0;

    g_head = head;
    g_bytes = bytes; g_ferr = ferr; g_perr = perr; g_drop = drop;

    // From the first start bit: maxBytes frames, start bit included, and one
    // bit of rest after the last so that its stop bit is seen to end.
    uint32_t shown = n;
    if (g_byteN) {
        uint32_t frameBits = (uint32_t)candBits(CAND[g_fmtIdx]) + 1u;
        shown = g_byteAt[0] + (((uint32_t)maxBytes * frameBits + 1u) * g_bitQ8 >> 8);
        if (shown > n) shown = n;
    }

    while (g_byteN && (g_byteN > maxBytes || g_byteAt[g_byteN - 1] >= shown)) g_byteN--;
    while (g_markN && g_marks[g_markN - 1] >= shown) g_markN--;
    latchView();
    g_markN = 0; g_byteN = 0;
    return (uint16_t)shown;
}

uint8_t  markCount() { return v_markN; }
uint16_t markAt(uint8_t i) { return i < v_markN ? v_marks[i] : 0; }

uint8_t  byteCount() { return v_byteN; }
uint16_t byteAt(uint8_t i)  { return i < v_byteN ? v_byteAt[i]  : 0; }
uint8_t  byteVal(uint8_t i) { return i < v_byteN ? v_byteVal[i] : 0; }

}
