// WedgeTest.cpp — minimal encoder-wedge bench harness.
//
// Distilled to ONLY the two things that demonstrably reduced the Nezha encoder
// wedge, with everything that didn't help removed (no modes, no bus/gap/reset/
// settle knobs):
//
//   1. Read BOTH encoders every tick, motor 1 (right) FIRST, then motor 2.
//   2. Write the motor command ONLY when the commanded speed changes.
//
// The loop is an explicit fixed-rate scheduler: each tick fires when the clock
// passes the next deadline (deadline = tick-start + period). The actual loop
// rate is measured and printed once a second so the cadence is verified — you
// can see for certain whether it is really running at 10 Hz (or whatever).
//
// Stops on any serial byte, or when a wedge is detected.
//
// Nezha V2 wire protocol (8-byte writes to I2C 0x10):
//   move/coast : FF F9 <id> <dir> 60 <speed 0-100> F5 00     (dir CW=1, CCW=2)
//   enc read   : FF F9 <id> 00 46 00 F5 00  then read 4 bytes (LE int32, 0.1 deg)
// Motor ids: right = 1, left = 2.

#include "WedgeTest.h"
#include "MicroBit.h"
#include "Robot.h"        // real-control mode drives through the production stack
#include "MotorController.h"
#include <cstdio>
#include <cstdint>

namespace {

constexpr uint16_t ADDR_W    = 0x10 << 1;   // 8-bit wire address
constexpr uint8_t  M_RIGHT   = 1;
constexpr uint8_t  M_LEFT    = 2;
constexpr uint8_t  DIR_CW    = 1;
constexpr uint8_t  DIR_CCW   = 2;
// Only the POST-write settle is kept (the chip needs time to prepare its 4-byte
// response). The old PRE-idle wait is dropped: the fixed-rate loop already
// leaves the bus idle between ticks, so it was redundant.
constexpr uint32_t SETTLE_US = 4000;        // post-0x46-write settle before the read

inline uint64_t nowUs() { return system_timer_current_time_us(); }

inline void busyUs(uint32_t us)
{
    uint64_t deadline = nowUs() + us;
    while (nowUs() < deadline) {}
}

// One 0x60 motor command. pct in [-100, 100]; 0 = coast.
void writeMotor(MicroBitI2C& i2c, uint8_t id, int pct)
{
    if (pct >  100) pct =  100;
    if (pct < -100) pct = -100;
    uint8_t dir = (pct >= 0) ? DIR_CW : DIR_CCW;
    uint8_t sp  = (pct >= 0) ? (uint8_t)pct : (uint8_t)(-pct);
    uint8_t buf[8] = { 0xFF, 0xF9, id, dir, 0x60, sp, 0xF5, 0x00 };
    i2c.write(ADDR_W, buf, 8, false);
}

// One encoder read from register `reg`: write the request, wait the post-write
// settle, read 4 bytes. reg = 0x46 (read-angle/position, what the firmware uses)
// or 0x47 (read-speed, the chip's native velocity — testing whether it wedges).
int32_t readEnc(MicroBitI2C& i2c, uint8_t id, uint8_t reg = 0x46)
{
    uint8_t cmd[8] = { 0xFF, 0xF9, id, 0x00, reg, 0x00, 0xF5, 0x00 };
    i2c.write(ADDR_W, cmd, 8, false);
    busyUs(SETTLE_US);
    uint8_t r[4] = { 0, 0, 0, 0 };
    i2c.read(ADDR_W, r, 4, false);
    return (int32_t)(((uint32_t)r[3] << 24) | ((uint32_t)r[2] << 16) |
                     ((uint32_t)r[1] << 8) | (uint32_t)r[0]);
}

// Sensor bus traffic — mimic the production load on the SHARED I2C bus: the
// OTOS (0x17), colour (0x1A) and line (0x43) sensors are read continuously by
// the telemetry/odometry path, interleaved with the motor 0x10 transactions.
// The motor-only harness never had this; production does. Exact registers don't
// matter — what matters is OTHER-device transactions landing between the motor
// write and the encoder read. A periodic write to 0x17 mimics the OV pose push.
void sensorTraffic(MicroBitI2C& i2c, uint32_t tick)
{
    uint8_t reg, buf[8];
    // OTOS 0x17 — pose-ish multi-byte read.
    reg = 0x20; i2c.write(0x17 << 1, &reg, 1, false); i2c.read(0x17 << 1, buf, 6, false);
    // Colour 0x1A — 2-byte read.
    reg = 0x09; i2c.write(0x1A << 1, &reg, 1, false); i2c.read(0x1A << 1, buf, 2, false);
    // Line 0x43 — 1-byte read.
    reg = 0x00; i2c.write(0x43 << 1, &reg, 1, false); i2c.read(0x43 << 1, buf, 1, false);
    // OV-like pose write to OTOS every ~20 ticks.
    if ((tick % 20) == 0) {
        uint8_t ov[7] = { 0x10, 0, 0, 0, 0, 0, 0 };
        i2c.write(0x17 << 1, ov, 7, false);
    }
}

constexpr int32_t JUMP = 4000;          // |delta| above this = implausible read
constexpr int     RETRIES = 6;          // re-reads to confirm a suspicious value

// Read an encoder, but if the value is an implausible jump from `prev`, RE-READ
// up to RETRIES times. If any re-read returns a sane value (the encoder is still
// counting), that's a transient glitch — count it and return the good value. If
// every re-read is still implausible, the encoder has truly latched: set
// *latched and return the (bad) value.
int32_t readConfirmed(MicroBitI2C& i2c, uint8_t id, int32_t prev,
                      uint32_t& glitches, bool& latched, uint8_t reg = 0x46)
{
    int32_t e = readEnc(i2c, id, reg);
    int32_t d = e - prev;
    if (d <= JUMP && d >= -JUMP) return e;          // looks normal — done
    for (int k = 0; k < RETRIES; ++k) {             // suspicious — confirm
        int32_t e2 = readEnc(i2c, id, reg);
        int32_t d2 = e2 - prev;
        if (d2 <= JUMP && d2 >= -JUMP) { ++glitches; return e2; }  // recovered
        e = e2;
    }
    latched = true;                                 // never recovered — real wedge
    return e;
}

// Drive pattern: hold (l, r) for `ticks` ticks, then advance to the next phase.
//
// IMPORTANT: l/r are PWM PERCENT (0-100), fed straight to writeMotor/0x60 — NOT
// mm/s. Production's velocity PID outputs only ~15-40% PWM for the speeds it
// drives; the OLD phases here used 90-120 (i.e. ~90-100% PWM, full throttle),
// which spun up instantly and never reproduced the wedge. These values are set
// to PRODUCTION PWM LEVELS so the one-wheel spin-up from a standstill is slow,
// like the real thing.
//
// Structure (what the user asked for): a defined stretch of NORMAL both-wheel
// operation, then the suspected TRIGGER — a ONE-WHEEL maneuver from a stop (one
// wheel driven, the other commanded 0). Every production wedge was a one-wheel
// maneuver (stand_soak "one L180 R0", "one R120 L0"); this isolates it.
// l/r units: mm/s in real-control mode (setTarget → PID), raw PWM% otherwise.
struct Phase { int l; int r; uint16_t ticks; const char* name; };
const Phase PHASES[] = {
    // --- a brief stretch of NORMAL both-wheel operation ---------------------
    { 120, 120,  60, "normal"   }, {   0,   0, 15, "stop" },
    // --- THE TRIGGER: SUSTAINED one-wheel maneuvers from a standstill. Long
    //     holds (200 ticks) so a freeze can LATCH instead of recovering at the
    //     next phase boundary (a transient EVT recovered on the short version).
    { 180,   0, 200, "ONE-L"    }, {   0,   0, 15, "stop" },  // left only,  mid speed
    {   0, 180, 200, "ONE-R"    }, {   0,   0, 15, "stop" },  // right only, mid speed
    {  60,   0, 200, "ONE-Lslo" }, {   0,   0, 15, "stop" },  // left only,  slow
    {   0,  60, 200, "ONE-Rslo" }, {   0,   0, 15, "stop" },  // right only, slow
};
constexpr int NPH = (int)(sizeof(PHASES) / sizeof(PHASES[0]));

}  // namespace

void runWedgeTest(MicroBit& uBit, int rateHz, int writeMs, int busKHz, int dither,
                  int reg, int sensors, int realCtrl, Robot* robot)
{
    MicroBitI2C& i2c = uBit.i2c;
    char line[240];

    if (rateHz < 1)   rateHz = 1;
    if (rateHz > 200) rateHz = 200;
    if (writeMs < 0)  writeMs = 0;
    if (busKHz < 50)  busKHz = 50;
    if (busKHz > 400) busKHz = 400;
    if (dither < 0)   dither = 0;
    uint8_t encReg = (reg == 0x47 || reg == 47) ? 0x47 : 0x46;  // per-tick read register
    bool useSensors = (sensors != 0);
    bool useReal    = (realCtrl != 0) && (robot != nullptr);    // drive via production PID path
    const uint32_t periodUs   = 1000000u / (uint32_t)rateHz;
    const uint32_t writeMinUs = (uint32_t)writeMs * 1000u;

    i2c.setFrequency((uint32_t)busKHz * 1000u);   // production bus = 400 kHz (restored at exit)

    snprintf(line, sizeof(line),
        "WEDGETEST start: realCtrl=%d reg=0x%02X sensors=%d @ %d Hz "
        "(period %u us, bus %d kHz, settle %u us, writeMin %d ms, dither +-%d). "
        "Any byte stops.\r\n",
        useReal ? 1 : 0, encReg, useSensors ? 1 : 0,
        rateHz, (unsigned)periodUs, busKHz, (unsigned)SETTLE_US, writeMs, dither);
    uBit.serial.send(line);

    // --- state ---
    int32_t prevR = readEnc(i2c, M_RIGHT, encReg);
    int32_t prevL = readEnc(i2c, M_LEFT, encReg);
    // Position snapshots (always 0x46) for a mode-independent wedge check at each
    // report: if commanded but position did NOT advance over a whole report
    // interval, the chip is wedged — works whether per-tick reads are 0x46 or 0x47.
    int32_t posPrevR = readEnc(i2c, M_RIGHT, 0x46);
    int32_t posPrevL = readEnc(i2c, M_LEFT, 0x46);
    bool droveSinceReport = false;
    int lastWrR = 0x7fff, lastWrL = 0x7fff; // last pwm actually WRITTEN (sentinel)
    uint64_t lastWriteUs = 0;               // for the write-min-interval rate limit
    int stuckR = 0, stuckL = 0;             // consecutive frozen (delta 0) reads
    int fwStuckMax = 0;                     // max firmware stuck counter seen (real mode)
    uint32_t glitches = 0;                  // transient bad reads that re-read OK
    uint32_t writes = 0;                    // motor writes issued (both wheels = 1)
    int pi = 0; uint16_t pk = 0;            // phase index, tick within phase
    uint32_t ticks = 0, cycles = 0;
    const char* verdict = "stopped";

    // --- explicit fixed-rate scheduler + rate measurement ---
    uint64_t nextTickUs   = nowUs();        // first tick fires immediately
    uint64_t reportAtUs   = nowUs() + 1000000u;
    uint32_t ticksAtReport = 0;
    uint32_t writesAtReport = 0;

    bool stop = false;
    while (!stop) {
        uint64_t now = nowUs();
        if (now < nextTickUs) {             // not time for the next tick yet
            if (uBit.serial.isReadable()) stop = true;
            continue;                       // spin until the deadline
        }
        nextTickUs = now + periodUs;        // schedule the next tick

        // ---- one tick: drive + read BOTH encoders --------------------------
        const Phase& ph = PHASES[pi];
        bool    latched = false;
        int32_t eR, eL;
        if (ph.r != 0 || ph.l != 0) droveSinceReport = true;

        if (useReal) {
            // === REAL production control path =================================
            // Set the mm/s target, then run the EXACT production control tick:
            // MotorController velocity PID → Motor::setSpeed (write-on-change +
            // rate-limit, via the I2CBus wrapper) → read BOTH encoders (M1 first)
            // with outlier rejection. This is what the soak does that the raw
            // path here does not. Phase l/r are mm/s in this mode.
            robot->motorController.setTarget((float)ph.l, (float)ph.r);
            if (useSensors) sensorTraffic(i2c, ticks);
            robot->controlCollectSplitPhase(now, 0);
            int32_t encL = 0, encR = 0;
            robot->motorController.getEncoderPositions(encL, encR);
            eR = encR; eL = encL;
        } else {
            // === RAW fixed-PWM path (the established-clean baseline) ==========
            // Mimic a PID-ish write rate by dithering the commanded PWM each tick
            // so write-on-change does not suppress it; write via raw uBit.i2c.
            int dv   = (dither > 0) ? ((ticks & 1) ? dither : -dither) : 0;
            int pwmR = (ph.r != 0) ? (ph.r + dv) : 0;
            int pwmL = (ph.l != 0) ? (ph.l + dv) : 0;
            bool changed  = (pwmR != lastWrR || pwmL != lastWrL);
            bool stopping = (pwmR == 0 && pwmL == 0);
            if (changed && (stopping || writeMinUs == 0 || (now - lastWriteUs) >= writeMinUs)) {
                writeMotor(i2c, M_RIGHT, pwmR);
                writeMotor(i2c, M_LEFT,  pwmL);
                lastWrR = pwmR; lastWrL = pwmL; lastWriteUs = now;
                ++writes;
            }
            if (useSensors) sensorTraffic(i2c, ticks);
            eR = readConfirmed(i2c, M_RIGHT, prevR, glitches, latched, encReg);
            eL = readConfirmed(i2c, M_LEFT,  prevL, glitches, latched, encReg);
        }
        ++ticks;

        // ---- wedge detection ----
        // (a) latched: a glitch value that never recovered across the re-reads.
        // (b) frozen: per-tick delta==0 while commanded — ONLY meaningful for 0x46
        //     (position). For 0x47 (speed) a steady phase is legitimately constant,
        //     so the per-tick frozen check is skipped and the report-time 0x46
        //     position check (below) is the wedge detector instead.
        int32_t dR = eR - prevR, dL = eL - prevL;
        prevR = eR; prevL = eL;
        if (encReg == 0x46) {
            if (ph.r != 0 && dR == 0) ++stuckR; else stuckR = 0;
            if (ph.l != 0 && dL == 0) ++stuckL; else stuckL = 0;
        }
        // In real-control mode the authoritative wedge signal is the firmware's
        // own stuck counter (it sees the raw pre-outlier-rejection reads). EVT
        // enc_wedged fires at 10; a PERSISTENT latch climbs much higher, so trip
        // at 40 to distinguish a real latch from a transient freeze that recovers.
        int fwStuck = 0;
        if (useReal && robot) {
            int sL = robot->motorController.stuckCountL();
            int sR = robot->motorController.stuckCountR();
            fwStuck = (sL > sR) ? sL : sR;
            if (fwStuck > fwStuckMax) fwStuckMax = fwStuck;
        }
        if (latched || stuckR >= 30 || stuckL >= 30 || fwStuck >= 40) {
            verdict = latched ? "WEDGE-LATCHED"
                     : (fwStuck >= 40 ? "WEDGE-FW-STUCK" : "WEDGE-FROZEN");
            snprintf(line, sizeof(line),
                "%s at tick=%lu cyc=%lu writes=%lu phase=%s  R=%ld L=%ld "
                "stuckR=%d stuckL=%d fwStuck=%d (after %lu glitches)\r\n",
                verdict, (unsigned long)ticks, (unsigned long)cycles,
                (unsigned long)writes, ph.name,
                (long)eR, (long)eL, stuckR, stuckL, fwStuck, (unsigned long)glitches);
            uBit.serial.send(line);
            break;
        }

        // ---- advance the drive pattern ----
        if (++pk >= ph.ticks) { pk = 0; if (++pi >= NPH) { pi = 0; ++cycles; } }

        // ---- verify the rate + mode-independent wedge check, once a second ----
        if (now >= reportAtUs) {
            uint32_t dticks  = ticks - ticksAtReport;     // ticks in the last ~1 s
            uint32_t dwrites = writes - writesAtReport;    // motor writes in the last ~1 s
            // Position check: did we drive but the position NOT move? Use the
            // production encoders in real-control mode (don't inject a raw read
            // into that path); a direct 0x46 read in the raw path.
            int32_t posR, posL;
            if (useReal) {
                int32_t posLtmp = 0, posRtmp = 0;
                robot->motorController.getEncoderPositions(posLtmp, posRtmp);
                posR = posRtmp; posL = posLtmp;
            } else {
                posR = readEnc(i2c, M_RIGHT, 0x46);
                posL = readEnc(i2c, M_LEFT, 0x46);
            }
            bool posStuck = droveSinceReport && (posR == posPrevR) && (posL == posPrevL);
            snprintf(line, sizeof(line),
                "  rate=%lu Hz  writes=%lu/s  cyc=%lu  glitches=%lu  fwStuckMax=%d  read(0x%02X)=%ld,%ld  pos=%ld,%ld\r\n",
                (unsigned long)dticks, (unsigned long)dwrites, (unsigned long)cycles,
                (unsigned long)glitches, fwStuckMax, encReg, (long)eR, (long)eL, (long)posR, (long)posL);
            uBit.serial.send(line);
            if (posStuck) {
                verdict = "WEDGE-POS-FROZEN";
                snprintf(line, sizeof(line),
                    "%s at tick=%lu cyc=%lu writes=%lu  pos=%ld,%ld (unchanged over 1s "
                    "while driving)  read(0x%02X)=%ld,%ld\r\n",
                    verdict, (unsigned long)ticks, (unsigned long)cycles,
                    (unsigned long)writes, (long)posR, (long)posL, encReg,
                    (long)eR, (long)eL);
                uBit.serial.send(line);
                break;
            }
            posPrevR = posR; posPrevL = posL; droveSinceReport = false;
            ticksAtReport  = ticks;
            writesAtReport = writes;
            reportAtUs += 1000000u;
        }
    }

    writeMotor(i2c, M_RIGHT, 0);
    writeMotor(i2c, M_LEFT, 0);
    i2c.setFrequency(400000);               // restore default bus for the rest of the firmware
    snprintf(line, sizeof(line),
             "WEDGETEST end (%s, %lu ticks, %lu writes, %lu cycles, %lu glitches)\r\n",
             verdict, (unsigned long)ticks, (unsigned long)writes,
             (unsigned long)cycles, (unsigned long)glitches);
    uBit.serial.send(line);
}
