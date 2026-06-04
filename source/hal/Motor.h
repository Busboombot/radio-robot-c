#pragma once
#include "MicroBit.h"
#include "Config.h"

/**
 * Motor — I2C driver for one channel of the PlanetX Nezha V2 motor controller.
 *
 * I2C address: 0x10 (7-bit).
 *
 * Protocol verified against PlanetX pxt-nezha2/main.ts:
 *   Motor start (8-byte write):
 *     [0xFF, 0xF9, motorId, direction, 0x60, speed, 0xF5, 0x00]
 *     direction: 1=CW (forward from chip perspective), 2=CCW (reverse)
 *     speed: 0-100 (absolute)
 *
 *   Encoder read (8-byte write + 4-byte read):
 *     Write: [0xFF, 0xF9, motorId, 0x00, 0x46, 0x00, 0xF5, 0x00]
 *     Read:  4 bytes, signed int32 little-endian, units = tenths of degrees
 *
 *   Encoder zero is maintained in software (offset scalar), matching the
 *   TypeScript resetRelAngleValue() behaviour.
 *
 * Constructor args:
 *   motorId  — 1 = M1 (right wheel), 2 = M2 (left wheel)
 *   fwdSign  — +1 or -1; maps the logical "forward" command to chip direction.
 *              Right wheel requires -1 because the motor is mounted mirrored.
 *
 * Vendor register coverage (all 9 vendor-documented registers):
 *   Register | HAL Method                             | Sprint | Status
 *   ---------|----------------------------------------|--------|-------
 *   0x60     | setSpeed()  — run motor at PWM %        | 008    | wrapped
 *   0x5F     | setSpeed(0) — stop motor                | 008    | wrapped
 *   0x46     | requestEncoder() / collectEncoder()     | 014    | split-phase
 *   (sw)     | resetEncoder()  — software offset zero  | 008    | wrapped
 *   0x47     | readSpeedRaw()  / readSpeed()           | 008    | wrapped
 *   0x70     | timedMove()  — timed/distance/turn move | 008    | wrapped
 *   0x5D     | moveToAngle() — absolute angle move     | 008    | wrapped
 *   0x1D     | resetHome()  — encoder/home zero        | 008    | wrapped
 *   0x77     | setGlobalSpeed() — global servo speed   | 008    | wrapped
 *   0x88     | readVersion() — firmware version        | 008    | wrapped
 */
class Motor {
public:
    Motor(MicroBitI2C& i2c, uint8_t motorId, int8_t fwdSign);

    // Set speed as signed percentage (-100..100). Positive = logical forward.
    // fwdSign is applied internally to map logical direction to chip direction.
    // Stores the commanded direction in _lastDir for readSpeed() sign inference.
    void    setSpeed(int8_t pct);

    // Read cumulative encoder in mm using calibration from cfg.
    // Uses mmPerDegL if motorId==LEFT_MOTOR, mmPerDegR otherwise.
    int32_t readEncoder(const RobotConfig& cfg) const;

    // High-resolution variant: cumulative encoder in mm as float (NOT truncated
    // to whole mm). Used by the velocity loop so the encoder-delta velocity
    // estimate isn't quantized to ±1 mm/tick (a throb source on the inner loop).
    float   readEncoderMmF(const RobotConfig& cfg) const;

    // Zero this motor's encoder accumulator (software offset reset,
    // matches chip TypeScript resetRelAngleValue() behaviour).
    void    resetEncoder();

    /**
     * requestEncoder — split-phase encoder I/O, phase 1.
     *
     * Issues the 0x46 write command and returns immediately (no busy-wait,
     * no fiber_sleep). The caller must ensure at least one full loop period
     * elapses before calling collectEncoder() — the cooperative loop's idle
     * sleep provides this guarantee. Only one wheel's request may be in
     * flight at a time; the LoopScheduler alternates wheels across ticks.
     */
    void requestEncoder();

    /**
     * collectEncoder — split-phase encoder I/O, phase 2.
     *
     * Reads back the 4-byte response issued by a prior requestEncoder() call
     * and returns the signed int32 (raw tenths of degrees minus _encOffset).
     * No busy-wait, no fiber_sleep. The caller is responsible for satisfying
     * the vendor's required inter-transaction delay (≥ one loop period,
     * supplied by the cooperative scheduler's idle sleep).
     */
    int32_t collectEncoder() const;

    /**
     * readSpeed — read chip-native wheel velocity.
     *
     * Issues a readSpeed command (register 0x47) and converts the raw uint16
     * reading to mm/s using:
     *   mm/s = (raw / kUnitFactor) * mmPerDeg * _lastDir
     *
     * where mmPerDeg = cfg.mmPerDegL (M2/left) or cfg.mmPerDegR (M1/right),
     * mirroring readEncoder()'s wheel-selection and calibration.
     *
     * kUnitFactor is a named constant in Motor.cpp (default 10.0 = tenths of
     * degrees/s, consistent with the 0x46 angle register).  After bench
     * confirmation, change kUnitFactor to 1.0 if raw is whole degrees/s.
     *
     * Sign convention: the chip reports unsigned magnitude only. Direction is
     * inferred from _lastDir (set by the most recent setSpeed() call). When
     * the motor is stopped (_lastDir == 0), velocity is reported as 0.
     *
     * Returns true on success; false if the I2C transaction fails (caller
     * should fall back to encoder-delta velocity).
     */
    bool readSpeed(float& mmPerSec, const RobotConfig& cfg) const;

    /**
     * timedMove — chip-controlled timed/distance/angle move (register 0x70).
     *
     * Frame (verified against pxt-nezha2/main.ts __move()):
     *   [0xFF, 0xF9, motorId, dir, 0x70, valueHigh, mode, valueLow]
     * where:
     *   dir   — 1=CW, 2=CCW (MovementDirection enum in vendor TS)
     *   value — int16 move amount (big-endian across buf[5] and buf[7])
     *   mode  — 1=turns, 2=degrees, 3=seconds (SportsMode enum in vendor TS)
     *
     * Note: value bytes are NOT contiguous in the frame — high byte at [5],
     * mode at [6], low byte at [7]. This matches the vendor TS exactly.
     *
     * Not wired into DriveController — provided for completeness and demos.
     */
    void timedMove(uint8_t dir, int16_t value, uint8_t mode);

    /**
     * moveToAngle — move motor to absolute angle (register 0x5D).
     *
     * Frame (verified against pxt-nezha2/main.ts moveToAbsAngle()):
     *   [0xFF, 0xF9, motorId, 0x00, 0x5D, angleHigh, mode, angleLow]
     * where:
     *   angle — 0-359 degrees (big-endian across buf[5] and buf[7])
     *   mode  — 1=shortest path, 2=CW, 3=CCW (ServoMotionMode enum)
     *
     * POST-WRITE DELAY (BUG-CRITICAL): The vendor comment says:
     *   "等待不能删除，且禁止有其他任务插入，否则有BUG"
     *   Translation: "The wait cannot be deleted and no other tasks are
     *   allowed to interleave, otherwise there will be a BUG."
     *
     * Resolution: We use a busy-wait loop (~4 ms), NOT fiber_sleep(4).
     * fiber_sleep() yields to the CODAL scheduler, which may dispatch
     * another fiber that issues an I2C transaction before this one
     * has been fully processed by the chip — exactly the interleave the
     * vendor warns against. The busy-wait keeps the CPU spinning for the
     * full 4 ms with no scheduler yield, guaranteeing no I2C interleave.
     *
     * Not wired into DriveController — provided for completeness and demos.
     */
    void moveToAngle(uint16_t angle, uint8_t mode);

    /**
     * resetHome — reset motor encoder/home position to zero (register 0x1D).
     *
     * Frame (verified against pxt-nezha2/main.ts reset()):
     *   [0xFF, 0xF9, motorId, 0x00, 0x1D, 0x00, 0xF5, 0x00]
     *
     * Note: vendor reset() also calls motorDelay(1, Second) — that delay is
     * for the motor to physically reach the home position. Callers should wait
     * at least 1 s before issuing further move commands after resetHome().
     */
    void resetHome();

    /**
     * setGlobalSpeed — set global servo speed for timed/angle moves (register 0x77).
     *
     * Frame (verified against pxt-nezha2/main.ts setServoSpeed()):
     *   [0xFF, 0xF9, 0x00, 0x00, 0x77, speedEncHigh, 0x00, speedEncLow]
     * where speedEnc = speed * 9 (range 0–900 for speed 0–100%).
     *
     * Note: motorId field (buf[2]) is 0x00 — this is a board-global command.
     * The speed encoding matches the vendor TS exactly: speed *= 9.
     */
    void setGlobalSpeed(uint8_t speed);

    /**
     * readVersion — read firmware version from the Nezha2 chip (register 0x88).
     *
     * Frame (verified against pxt-nezha2/main.ts readVersion()):
     *   Write: [0xFF, 0xF9, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00]
     *   Read:  3 bytes [major, minor, patch]
     *
     * Note: unlike most read commands, buf[6] is 0x00 (not 0xF5) and
     * motorId (buf[2]) is 0x00 — board-global command.
     *
     * Returns true on success, false on I2C error.
     */
    bool readVersion(uint8_t& maj, uint8_t& min, uint8_t& patch);

    /**
     * readEncoderAtomic — safe single-shot encoder read (raw tenths-of-degrees).
     *
     * Implements the full vendor pxt-nezha2 readAngle() timing (sprint 013
     * readEncoderRaw() pattern):
     *   4 ms pre-write bus-idle → 0x46 write → 4 ms post-write settle → read 4 bytes.
     *
     * Both delays are required (confirmed by sprint 013 bench):
     *   - pre-write: allows the I2C bus to idle after the previous transaction.
     *   - post-write: allows the chip to prepare its 4-byte response.
     * Busy-wait is used (NOT fiber_sleep) so the CODAL scheduler cannot
     * dispatch a competing I2C transaction during the window.
     *
     * Returns raw tenths-of-degrees minus the software offset (_encOffset).
     *
     * Use for: resetEncoder(), any one-off read outside the control tick.
     * Cost: ~8 ms (two 4ms delays).
     */
    int32_t readEncoderAtomic() const;

    /**
     * readEncoderMmFAtomic — safe single-shot encoder read in mm (float).
     *
     * Same as readEncoderAtomic() but converts to mm using calibration from cfg.
     * Use for: startDrive(), startDriveClean(), stop() — any position snapshot
     * outside the normal control tick.  Cost: ~8 ms.
     */
    float readEncoderMmFAtomic(const RobotConfig& cfg) const;

private:
    MicroBitI2C& _i2c;
    uint8_t      _motorId;  // 1=M1/right, 2=M2/left
    int8_t       _fwdSign;  // +1 or -1

    // Commanded direction: +1 = logical forward, -1 = logical reverse, 0 = stopped.
    // Set by setSpeed(); read by readSpeed() to apply sign to the unsigned chip reading.
    int8_t _lastDir;

    static constexpr uint8_t ADDR    = 0x10;
    static constexpr uint8_t DIR_CW  = 1;   // positive speed from chip perspective
    static constexpr uint8_t DIR_CCW = 2;   // negative speed from chip perspective

    // Software encoder offset (tenths of degrees), zeroed by resetEncoder().
    mutable int32_t _encOffset;

    // Write an 8-byte motor command to the chip.
    void    writeMotorCmd(uint8_t direction, uint8_t speed);

    // Legacy synchronous encoder read (write + immediate read, no busy-wait).
    // No longer used by resetEncoder() (replaced by readEncoderAtomic()).
    // Retained for reference; can be removed once confirmed unnecessary.
    int32_t readEncoderRaw() const;

    // Read raw speed from chip register 0x47 (uint16 LE, unsigned magnitude).
    // Returns -1 on I2C error.
    int32_t readSpeedRaw() const;
};
