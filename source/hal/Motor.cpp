#include "Motor.h"
#include <math.h>

// ---------------------------------------------------------------------------
// I2C wire protocol constants (verified against PlanetX pxt-nezha2/main.ts)
// ---------------------------------------------------------------------------
//
// Every command is an 8-byte write to address 0x10.
// The frame always starts with 0xFF 0xF9 followed by motor-id, then
// a command-specific payload in bytes [3..7].
//
// Motor start (__start):
//   [0xFF, 0xF9, motorId, direction, 0x60, speed, 0xF5, 0x00]
//   direction: 1 = CW (positive speed), 2 = CCW (negative speed)
//   speed: absolute value 0-100
//
// Motor stop (stop):
//   [0xFF, 0xF9, motorId, 0x00, 0x5F, 0x00, 0xF5, 0x00]
//
// Encoder read (readAngle):
//   Write: [0xFF, 0xF9, motorId, 0x00, 0x46, 0x00, 0xF5, 0x00]
//   Read:  4 bytes, signed int32 little-endian, units = tenths of degrees
// ---------------------------------------------------------------------------

Motor::Motor(I2CBus& i2c, uint8_t motorId, int8_t fwdSign)
    : _i2c(i2c), _motorId(motorId), _fwdSign(fwdSign), _lastDir(0), _encOffset(0)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void Motor::setSpeed(int8_t pct)
{
    // Clamp to [-100, 100].
    if (pct >  100) pct =  100;
    if (pct < -100) pct = -100;

    // Write-on-change: skip the I2C write if the command is unchanged. The
    // control loop calls setSpeed() every tick (~100 Hz); writing the Nezha
    // that fast wedges its encoder reads (the read freezes at a constant while
    // the wheels keep spinning). Writing only on a real change keeps the
    // controller healthy. See docs/knowledge encoder-wedge note.
    if (pct == _lastWrittenPct) {
        return;
    }

    // Write-rate limit (sprint 015 — wedge root cause). The velocity PID emits
    // a slightly different PWM EVERY 10 ms tick, so plain write-on-change does
    // NOT suppress writes — the chip gets a fresh 0x60 write nearly every tick,
    // interleaved between the two 0x46 encoder reads. That high-frequency
    // write/read interleave is exactly what wedges the Nezha encoder readback
    // (proven: the WedgeTest harness holds a CONSTANT speed so its writes are
    // suppressed for 40-tick stretches → zero wedges over 10 min; the alternating
    // production path wedged in ~165 ticks). We throttle 0x60 writes to
    // kMinWriteIntervalUs so the bus is dominated by reads, matching WedgeTest —
    // BUT a stop (pct == 0) or a direction reversal is always written immediately
    // for safety/responsiveness. Between throttled writes the chip simply holds
    // the last 0x60 command (the wheels keep spinning), and the next allowed
    // write applies the freshest PID output.
    static constexpr uint32_t kMinWriteIntervalUs = 40000;   // 40 ms ≈ 25 Hz max
    bool stopping = (pct == 0);
    bool reversal = (pct != 0 && _lastWrittenPct != 0 &&
                     ((pct > 0) != (_lastWrittenPct > 0)));
    uint64_t nowUs = system_timer_current_time_us();
    if (!stopping && !reversal &&
        (nowUs - _lastWriteUs) < kMinWriteIntervalUs) {
        // Too soon since the last write and not a stop/reversal — suppress.
        // _lastWrittenPct is deliberately NOT updated, so the next tick still
        // sees a change and writes the latest PID output once the interval ends.
        return;
    }
    _lastWriteUs    = nowUs;
    _lastWrittenPct = pct;

    // Apply fwdSign: positive pct = logical forward; fwdSign maps that to
    // the chip's CW/CCW convention.  For the right wheel, fwdSign = -1 so
    // that a positive command results in CCW chip rotation (physical forward).
    int16_t effective = (int16_t)_fwdSign * (int16_t)pct;

    if (effective == 0) {
        // Zero speed: COAST via the 0x60 move command with speed 0 — NOT the
        // 0x5F "shutdown" command. 0x5F shuts the Nezha controller down and
        // wedges subsequent encoder reads (they freeze at a constant). The old
        // firmware used 0x60-speed-0 to coast and reserved 0x5F for a final
        // program-end stop. See docs/knowledge encoder-wedge note.
        writeMotorCmd(DIR_CW, 0);
        _lastDir = 0;
    } else {
        uint8_t dir   = (effective > 0) ? DIR_CW : DIR_CCW;
        uint8_t speed = (effective > 0) ? (uint8_t)effective : (uint8_t)(-effective);
        writeMotorCmd(dir, speed);
        // Track logical direction (sign of the original pct, not the chip direction)
        // so readSpeed() can apply the correct sign to the unsigned chip reading.
        _lastDir = (pct > 0) ? (int8_t)1 : (int8_t)-1;
    }
}

int32_t Motor::readEncoder(const RobotConfig& cfg) const
{
    // motorId 2 = M2 = left wheel; use mmPerDegL.
    // motorId 1 = M1 = right wheel; use mmPerDegR.
    float mmPerDeg = (_motorId == 2) ? cfg.mmPerDegL : cfg.mmPerDegR;

    // NOTE: split-phase contract — caller must have issued requestEncoder()
    // at least one loop period before calling this. collectEncoder() issues
    // the 4-byte read without any busy-wait. The loop's idle sleep provides
    // the required vendor inter-transaction delay.
    int32_t raw = collectEncoder();   // tenths of degrees
    // Mirror TypeScript: (raw / 10.0) * mmPerDeg * fwdSign
    float degF  = raw / 10.0f;
    float mmF   = degF * mmPerDeg * (float)_fwdSign;
    return (int32_t)mmF;
}

float Motor::readEncoderMmF(const RobotConfig& cfg) const
{
    // Same as readEncoder() but returns full float resolution (no truncation to
    // whole mm). The velocity loop differentiates position, so 1 mm truncation
    // becomes ±~17 mm/s quantization noise at the ~58 ms loop rate.
    //
    // NOTE: split-phase contract — caller must have issued requestEncoder()
    // at least one loop period before calling this. collectEncoder() issues
    // the 4-byte read without any busy-wait.
    float mmPerDeg = (_motorId == 2) ? cfg.mmPerDegL : cfg.mmPerDegR;
    int32_t raw = collectEncoder();   // tenths of degrees
    return (raw / 10.0f) * mmPerDeg * (float)_fwdSign;
}

void Motor::resetEncoder()
{
    // Mirror TypeScript resetRelAngleValue(): snapshot the current raw
    // angle into the software offset so that subsequent reads return zero.
    //
    // (033-005a) Median-of-3 snapshot + readback verification:
    //   1. Take three atomic reads and use the median as the offset delta.
    //      A single garbage read (e.g. the ~149 mm ZERO-enc offset corruption
    //      seen on bench, Robot.cpp:123-125) cannot skew the median.
    //   2. After updating _encOffset, verify that a fresh atomic read returns
    //      |result| < kReadbackThreshold (≈ 2 encoder counts).  If not, retry
    //      the whole snapshot up to kMaxRetries times.  This catches cases where
    //      a corrupted offset causes subsequent reads to be non-zero even after
    //      the reset (false "no motion" reads from the velocity loop).
    //
    // Cost: 3 atomic reads × ~8 ms each = ~24 ms per successful reset;
    //       up to kMaxRetries+1 attempts (rare: only on genuine I2C garbage).
    static constexpr int     kMaxRetries       = 2;
    static constexpr int32_t kReadbackThreshold = 2;  // tenths of degrees ≈ <1 mm

    for (int attempt = 0; attempt <= kMaxRetries; ++attempt) {
        // Median-of-3: take three reads and sort to find the middle value.
        int32_t s0 = readEncoderAtomic();
        int32_t s1 = readEncoderAtomic();
        int32_t s2 = readEncoderAtomic();

        // Three-element median (branchless sort not available in C++11 without
        // algorithm; use explicit comparisons that the compiler inlines well).
        int32_t lo = s0, mid = s1, hi = s2;
        if (lo > hi) { int32_t tmp = lo; lo = hi; hi = tmp; }
        if (lo > mid) { int32_t tmp = lo; lo = mid; mid = tmp; }
        if (mid > hi) { mid = hi; }
        int32_t snapshot = mid;

        _encOffset += snapshot;

        // Readback check: after the offset update, a fresh read should be ≈ 0.
        int32_t readback = readEncoderAtomic();
        if (readback >= -kReadbackThreshold && readback <= kReadbackThreshold) {
            // Clean reset — done.
            return;
        }
        // Readback non-zero: the offset snapshot was corrupted.  Undo this
        // attempt (restore _encOffset) and retry.
        _encOffset -= snapshot;
    }
    // All retries exhausted: apply the last snapshot anyway so the encoder is
    // at least approximately zero rather than leaving it completely uncorrected.
    int32_t s0 = readEncoderAtomic();
    int32_t s1 = readEncoderAtomic();
    int32_t s2 = readEncoderAtomic();
    int32_t lo = s0, mid = s1, hi = s2;
    if (lo > hi) { int32_t tmp = lo; lo = hi; hi = tmp; }
    if (lo > mid) { int32_t tmp = lo; lo = mid; mid = tmp; }
    if (mid > hi) { mid = hi; }
    _encOffset += mid;
}

int32_t Motor::readEncoderAtomic() const
{
    // Atomic single-wheel encoder read using the full vendor pxt-nezha2
    // readAngle() timing (matches sprint 013 readEncoderRaw()):
    //   4 ms pre-write bus-idle → 0x46 write → 4 ms post-write settle → read 4 bytes.
    //
    // Both delays are required:
    //   - pre-write: allows the I2C bus to idle after the previous transaction.
    //   - post-write: allows the chip to prepare its 4-byte response.
    // Busy-wait is used (NOT fiber_sleep) so the CODAL scheduler cannot dispatch
    // a competing I2C transaction during the window.
    //
    // Cost: ~8 ms per call — acceptable for one-off operations.
    static constexpr uint32_t kDelayUs = 4000;  // 4 ms each phase (vendor requirement)

    // Pre-write bus-idle delay.
    {
        uint64_t deadline = system_timer_current_time_us() + kDelayUs;
        while (system_timer_current_time_us() < deadline) {}
    }

    uint8_t cmd[8] = {
        0xFF, 0xF9,
        _motorId,
        0x00, 0x46,
        0x00, 0xF5,
        0x00
    };
    _i2c.write((ADDR << 1), (uint8_t*)cmd, 8, false);

    // Post-write settle: chip prepares the 4-byte encoder response.
    {
        uint64_t deadline = system_timer_current_time_us() + kDelayUs;
        while (system_timer_current_time_us() < deadline) {}
    }

    uint8_t resp[4] = {0, 0, 0, 0};
    _i2c.read((ADDR << 1), (uint8_t*)resp, 4, false);

    int32_t raw = (int32_t)(
        ((uint32_t)resp[3] << 24) |
        ((uint32_t)resp[2] << 16) |
        ((uint32_t)resp[1] <<  8) |
        ((uint32_t)resp[0])
    );

    // Subtract the software offset captured at last resetEncoder() call.
    return raw - _encOffset;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void Motor::writeMotorCmd(uint8_t direction, uint8_t speed)
{
    uint8_t buf[8] = {
        0xFF,
        0xF9,
        _motorId,
        direction,
        0x60,
        speed,
        0xF5,
        0x00
    };
    _i2c.write((ADDR << 1), (uint8_t*)buf, 8, false);
}

// ---------------------------------------------------------------------------
// Split-phase encoder I/O (ticket 014-002)
// ---------------------------------------------------------------------------

void Motor::requestEncoder()
{
    // Phase 1: Issue the 0x46 encoder command write and return immediately.
    //
    // NOTE: repeated=false (STOP after write) is used here. The read is done
    // via collectEncoder() as a separate transaction. If repeated-start is needed,
    // use readEncoderAtomic() which keeps the bus held.
    uint8_t cmd[8] = {
        0xFF, 0xF9,
        _motorId,
        0x00, 0x46,
        0x00, 0xF5,
        0x00
    };
    _i2c.write((ADDR << 1), (uint8_t*)cmd, 8, false);
}

int32_t Motor::collectEncoder() const
{
    // Phase 2: Read back the 4-byte encoder response.
    //
    // Must be called at least one loop period after requestEncoder() to
    // satisfy the vendor's required inter-transaction delay. The cooperative
    // loop guarantees this ordering (LoopScheduler alternates wheels).
    // No busy-wait or fiber_sleep.
    uint8_t resp[4] = {0, 0, 0, 0};
    _i2c.read((ADDR << 1), (uint8_t*)resp, 4, false);

    int32_t raw = (int32_t)(
        ((uint32_t)resp[3] << 24) |
        ((uint32_t)resp[2] << 16) |
        ((uint32_t)resp[1] <<  8) |
        ((uint32_t)resp[0])
    );

    // Subtract the software offset captured at last resetEncoder() call.
    return raw - _encOffset;
}

float Motor::readEncoderMmFAtomic(const RobotConfig& cfg) const
{
    // Atomic read in mm (float). Same conversion as readEncoderMmF() but using
    // readEncoderAtomic() so it is safe outside the control tick.
    float mmPerDeg = (_motorId == 2) ? cfg.mmPerDegL : cfg.mmPerDegR;
    int32_t raw = readEncoderAtomic();  // tenths of degrees minus offset
    return (raw / 10.0f) * mmPerDeg * (float)_fwdSign;
}

float Motor::readEncoderMmFSettle(const RobotConfig& cfg) const
{
    // Settle-only encoder read — skips the 4 ms pre-write bus-idle used by
    // readEncoderAtomic(). The fixed-rate control loop leaves the bus naturally
    // idle between ticks, so the pre-idle is redundant there. Cost: ~4 ms.
    static constexpr uint32_t kSettleUs = 4000;
    uint8_t cmd[8] = { 0xFF, 0xF9, _motorId, 0x00, 0x46, 0x00, 0xF5, 0x00 };
    _i2c.write((ADDR << 1), (uint8_t*)cmd, 8, false);
    uint64_t deadline = system_timer_current_time_us() + kSettleUs;
    while (system_timer_current_time_us() < deadline) {}
    uint8_t resp[4] = { 0, 0, 0, 0 };
    _i2c.read((ADDR << 1), (uint8_t*)resp, 4, false);
    int32_t raw = (int32_t)(
        ((uint32_t)resp[3] << 24) | ((uint32_t)resp[2] << 16) |
        ((uint32_t)resp[1] <<  8) | (uint32_t)resp[0]);
    raw -= _encOffset;
    float mmPerDeg = (_motorId == 2) ? cfg.mmPerDegL : cfg.mmPerDegR;
    return (raw / 10.0f) * mmPerDeg * (float)_fwdSign;
}

int32_t Motor::readEncoderRaw() const
{
    // Legacy synchronous encoder read — kept for callers that have not yet
    // migrated to the split-phase requestEncoder()/collectEncoder() API.
    // The busy-wait spin loops have been removed: in the cooperative single-
    // loop architecture the scheduler's idle sleep provides the required
    // inter-transaction delay. Ticket 003 will migrate all remaining callers
    // to the split-phase API, at which point this method can be removed.
    uint8_t cmd[8] = {
        0xFF, 0xF9,
        _motorId,
        0x00, 0x46,
        0x00, 0xF5,
        0x00
    };
    _i2c.write((ADDR << 1), (uint8_t*)cmd, 8, false);

    // Read 4 bytes (signed int32, little-endian).
    uint8_t resp[4] = {0, 0, 0, 0};
    _i2c.read((ADDR << 1), (uint8_t*)resp, 4, false);

    int32_t raw = (int32_t)(
        ((uint32_t)resp[3] << 24) |
        ((uint32_t)resp[2] << 16) |
        ((uint32_t)resp[1] <<  8) |
        ((uint32_t)resp[0])
    );

    // Subtract the software offset captured at last resetEncoder() call.
    return raw - _encOffset;
}

int32_t Motor::readSpeedRaw() const
{
    // Vendor pxt-nezha2 readSpeed() — register 0x47.
    // Frame: [0xFF, 0xF9, motorId, 0x00, 0x47, 0x00, 0xF5, 0x00]
    // Response: 2 bytes, unsigned uint16 little-endian.
    //
    // The chip returns unsigned speed magnitude; direction must be inferred
    // from the commanded PWM sign (_lastDir), not from this register.
    //
    // Timing rationale (sprint 012-004):
    //   The vendor oracle uses 4 ms pre + 4 ms post (identical to readEncoderRaw).
    //   In practice, register 0x47 is always read immediately after two 0x46 encoder
    //   reads in the tick() loop.  The chip appears to need additional time to switch
    //   internal register context and update the speed accumulator after the preceding
    //   encoder-read transactions.  Increasing the post-write delay from 4 ms to 8 ms
    //   gives the chip a longer window to prepare the speed response, which reduces
    //   the probability of reading a stale (stuck) value.
    //
    //   The pre-write delay remains 4 ms (matching vendor oracle) to allow the bus
    //   to idle after the previous transaction before we assert the 0x47 command.
    //
    //   HARDWARE-CONFIRM REQUIRED: After reflashing, run `S 200 200` and observe
    //   `GET VEL`.  If source flag is still 'E' (encoder fallback) due to stuck chip
    //   reading, try increasing kPostWriteDelayMs further (12 ms, then 16 ms).
    //   If source flag shows 'C' with plausible values (~200 mm/s), the fix is confirmed.
    //   The Part-A plausibility gate (stuck-low rejection) provides a safety net
    //   during this hardware confirmation phase.
    // IMPORTANT: Use busy-wait spins rather than fiber_sleep — same rationale
    // as readEncoderRaw(): fiber_sleep() yields to the CODAL scheduler and
    // allows the comms fiber to issue an I2C write mid-transaction, corrupting
    // the speed register read.  Busy-wait keeps the transaction atomic.
    static constexpr uint32_t kPreWriteDelayUs  = 4000;   // 4 ms
    static constexpr uint32_t kPostWriteDelayUs = 8000;   // 8 ms (increased from 4 ms, 012-004)

    {
        uint64_t deadline = system_timer_current_time_us() + kPreWriteDelayUs;
        while (system_timer_current_time_us() < deadline) {}
    }
    uint8_t cmd[8] = {
        0xFF, 0xF9,
        _motorId,
        0x00, 0x47,
        0x00, 0xF5,
        0x00
    };
    int writeResult = _i2c.write((ADDR << 1), (uint8_t*)cmd, 8, false);
    {
        uint64_t deadline = system_timer_current_time_us() + kPostWriteDelayUs;
        while (system_timer_current_time_us() < deadline) {}
    }

    if (writeResult != MICROBIT_OK) {
        return -1;  // I2C error sentinel
    }

    // Read 2 bytes (unsigned uint16 LE).
    uint8_t resp[2] = {0, 0};
    int readResult = _i2c.read((ADDR << 1), (uint8_t*)resp, 2, false);

    if (readResult != MICROBIT_OK) {
        return -1;  // I2C error sentinel
    }

    uint16_t raw = (uint16_t)(((uint16_t)resp[1] << 8) | (uint16_t)resp[0]);
    return (int32_t)raw;
}

// ---------------------------------------------------------------------------
// Additional vendor register wrappers (ticket 008-004)
// ---------------------------------------------------------------------------

void Motor::timedMove(uint8_t dir, int16_t value, uint8_t mode)
{
    // Frame verified against pxt-nezha2/main.ts __move():
    //   buf[0]=0xFF, buf[1]=0xF9, buf[2]=motorId, buf[3]=direction,
    //   buf[4]=0x70, buf[5]=(value>>8)&0xFF, buf[6]=mode, buf[7]=(value>>0)&0xFF
    //
    // value is encoded big-endian with mode interleaved at buf[6].
    // This layout is NOT a standard little-endian int16; it matches the vendor
    // TypeScript byte-for-byte.
    uint8_t buf[8] = {
        0xFF, 0xF9,
        _motorId,
        dir,
        0x70,
        (uint8_t)((uint16_t)value >> 8),    // high byte
        mode,                                // mode at buf[6]
        (uint8_t)((uint16_t)value & 0xFF)   // low byte at buf[7]
    };
    _i2c.write((ADDR << 1), (uint8_t*)buf, 8, false);
}

void Motor::moveToAngle(uint16_t angle, uint8_t mode)
{
    // Clamp to 0-359 (mirrors vendor TS: angle %= 360).
    angle = angle % 360;

    // Frame verified against pxt-nezha2/main.ts moveToAbsAngle():
    //   buf[0]=0xFF, buf[1]=0xF9, buf[2]=motorId, buf[3]=0x00,
    //   buf[4]=0x5D, buf[5]=(angle>>8)&0xFF, buf[6]=turnMode, buf[7]=(angle>>0)&0xFF
    //
    // Same interleaved layout as 0x70: high byte at [5], mode at [6], low byte at [7].
    uint8_t buf[8] = {
        0xFF, 0xF9,
        _motorId,
        0x00,
        0x5D,
        (uint8_t)(angle >> 8),    // high byte
        mode,                     // ServoMotionMode at buf[6]
        (uint8_t)(angle & 0xFF)   // low byte at buf[7]
    };
    _i2c.write((ADDR << 1), (uint8_t*)buf, 8, false);

    // BUG-CRITICAL: 4 ms post-write busy-wait (no task/fiber interleave).
    //
    // Vendor comment: "等待不能删除，且禁止有其他任务插入，否则有BUG"
    // Translation: "The wait cannot be deleted and no other tasks are
    // allowed to interleave, otherwise there will be a BUG."
    //
    // We use a busy-wait (spin on system_timer_current_time_us()) rather
    // than fiber_sleep(4).  fiber_sleep() yields control to the CODAL
    // scheduler, which can dispatch another fiber that issues its own I2C
    // write before the chip has finished processing the 0x5D command —
    // exactly the interleave the vendor warns against.  The busy-wait
    // holds the CPU for ~4 ms with no scheduler switch, guaranteeing the
    // 0x5D command completes in isolation.
    uint64_t deadline = system_timer_current_time_us() + 4000;  // 4 ms
    while (system_timer_current_time_us() < deadline) {
        // busy-wait — intentionally does not yield
    }
}

void Motor::resetHome()
{
    // Frame verified against pxt-nezha2/main.ts reset():
    //   [0xFF, 0xF9, motorId, 0x00, 0x1D, 0x00, 0xF5, 0x00]
    //
    // The vendor also resets relativeAngularArr[motor-1] = 0 and calls
    // motorDelay(1, Second) to allow the motor to physically reach home.
    // Callers should wait ≥1 s before issuing further move commands.
    uint8_t buf[8] = {
        0xFF, 0xF9,
        _motorId,
        0x00, 0x1D,
        0x00, 0xF5,
        0x00
    };
    _i2c.write((ADDR << 1), (uint8_t*)buf, 8, false);
}

void Motor::setGlobalSpeed(uint8_t speed)
{
    // Frame verified against pxt-nezha2/main.ts setServoSpeed():
    //   speedEnc = speed * 9   (0–900 for speed 0–100%)
    //   [0xFF, 0xF9, 0x00, 0x00, 0x77, speedEncHigh, 0x00, speedEncLow]
    //
    // motorId (buf[2]) is 0x00 — this is a board-global command affecting
    // all motor channels.  buf[6] is 0x00, not 0xF5.
    if (speed > 100) speed = 100;
    uint16_t speedEnc = (uint16_t)speed * 9;   // 0–900

    uint8_t buf[8] = {
        0xFF, 0xF9,
        0x00,                              // board-global, not per-motor
        0x00,
        0x77,
        (uint8_t)(speedEnc >> 8),          // high byte
        0x00,                              // buf[6] = 0x00 per vendor TS
        (uint8_t)(speedEnc & 0xFF)         // low byte
    };
    _i2c.write((ADDR << 1), (uint8_t*)buf, 8, false);
}

bool Motor::readVersion(uint8_t& maj, uint8_t& min, uint8_t& patch)
{
    // Frame verified against pxt-nezha2/main.ts readVersion():
    //   Write: [0xFF, 0xF9, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00]
    //   Read:  3 bytes [major, minor, patch]
    //
    // motorId (buf[2]) is 0x00 — board-global command.
    // buf[6] is 0x00 (NOT 0xF5 as used in motor-specific read commands).
    // No pre/post delay in the vendor TS for this command.
    uint8_t cmd[8] = {
        0xFF, 0xF9,
        0x00, 0x00,
        0x88,
        0x00, 0x00, 0x00
    };
    int writeResult = _i2c.write((ADDR << 1), (uint8_t*)cmd, 8, false);
    if (writeResult != MICROBIT_OK) {
        maj = min = patch = 0;
        return false;
    }

    uint8_t resp[3] = {0, 0, 0};
    int readResult = _i2c.read((ADDR << 1), (uint8_t*)resp, 3, false);
    if (readResult != MICROBIT_OK) {
        maj = min = patch = 0;
        return false;
    }

    maj   = resp[0];
    min   = resp[1];
    patch = resp[2];
    return true;
}

bool Motor::readSpeed(float& mmPerSec, const RobotConfig& cfg) const
{
    int32_t raw = readSpeedRaw();
    if (raw < 0) {
        // I2C error — caller should fall back to encoder-delta velocity.
        mmPerSec = 0.0f;
        return false;
    }

    // Convert register 0x47 raw value to mm/s.
    //
    // The 0x47 register reports angular velocity in the SAME unit as the
    // 0x46 angle register: tenths of degrees.  This mirrors readEncoder(),
    // which converts 0x46 raw via: mm = (raw / 10.0) * mmPerDeg * fwdSign.
    //
    // Therefore: mm/s = (raw / 10.0) * mmPerDeg * sign
    //
    // motorId 2 = M2 = left wheel; use mmPerDegL.
    // motorId 1 = M1 = right wheel; use mmPerDegR.
    //
    // BENCH-CONFIRM REQUIRED: The vendor TypeScript formula treats the raw
    // value as whole degrees/s (not tenths), which contradicts the 0x46
    // register documentation.  The /10 (tenths) interpretation is used here
    // because it is consistent with the 0x46 register unit.
    //
    // To verify: drive at a steady speed (e.g. S 200 200), compare
    // readSpeed() mm/s to encoder-delta mm/s:
    //   - If readSpeed is ~10× encoder-delta → /10 is correct (keep kUnitFactor = 10.0f)
    //   - If readSpeed matches encoder-delta  → raw is whole deg/s (set kUnitFactor = 1.0f)
    //
    // Change kUnitFactor to flip the interpretation after bench confirmation.
    static constexpr float kUnitFactor = 10.0f;  // BENCH-CONFIRM: 10.0 = tenths; 1.0 = whole deg/s

    float mmPerDeg = (_motorId == 2) ? cfg.mmPerDegL : cfg.mmPerDegR;
    float magnitude = ((float)raw / kUnitFactor) * mmPerDeg;

    // Apply direction sign: the chip returns unsigned speed only.
    // _lastDir is +1 (forward), -1 (reverse), or 0 (stopped).
    mmPerSec = magnitude * (float)_lastDir;
    return true;
}
