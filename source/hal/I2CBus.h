#pragma once
#include "MicroBit.h"
#include <stdint.h>

/**
 * I2CBus — thin diagnostic wrapper around MicroBitI2C.
 *
 * All four I2C device classes (Motor, OtosSensor, LineSensor, ColorSensor)
 * route their bus traffic through this wrapper so every transaction is
 * counted, errors are tracked, and potential re-entrancy violations are
 * captured — without changing the semantics of any transaction.
 *
 * Re-entrancy guard (diagnostic, NOT a lock):
 *   _inUse is checked and set atomically via target_disable_irq() /
 *   target_enable_irq() around the flag access only (NOT around the full
 *   I2C transaction). If a second call enters while the first is still in
 *   flight (e.g. an ISR-context call or a spurious re-entry), the violation
 *   counter is incremented and the address pair is captured. The transaction
 *   proceeds normally — the guard records the concurrency violation but does
 *   NOT block or skip any I2C traffic.
 *
 * Per-device counters:
 *   Keyed by the 7-bit device address (the address before the caller
 *   left-shifts it). Known devices:
 *     0x10 — Nezha V2 motor controller (Motor)
 *     0x17 — OTOS odometry sensor (OtosSensor)
 *     0x1A — line sensor (LineSensor)
 *     0x39 — APDS9960 color sensor (ColorSensor, fallback variant)
 *     0x43 — alt color sensor (ColorSensor, primary variant)
 *   Unrecognised addresses are accumulated in an "other" bucket (index
 *   kMaxDevices-1) so the table never overflows.
 *
 * Usage (source/main.cpp):
 *   static I2CBus bus(uBit.i2c);  // one instance, before all device objects
 *   static Motor motorL(bus, 2, cfg.fwdSignL);
 *   ...
 *
 * Thread safety:
 *   The _inUse flag window is intentionally narrow (3–4 instructions) so the
 *   critical section is safe on Cortex-M4. All counter increments happen
 *   outside IRQ-disabled sections and are therefore NOT ISR-safe themselves —
 *   they are meant for cooperative-loop diagnostics only.
 */
class I2CBus {
public:
    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    explicit I2CBus(MicroBitI2C& bus);

    // -----------------------------------------------------------------------
    // I2C forwarding — mirror MicroBitI2C signatures exactly.
    //
    // address: 8-bit wire address (7-bit addr << 1), as the callers pass it.
    // Returns: CODAL status int (MICROBIT_OK == 0 on success).
    // -----------------------------------------------------------------------

    int write(uint16_t address, uint8_t* data, int len, bool repeated = false);
    int read (uint16_t address, uint8_t* data, int len, bool repeated = false);

    // -----------------------------------------------------------------------
    // Per-device statistics (keyed by 7-bit address)
    // -----------------------------------------------------------------------

    /** Total transactions (write + read) for the device at addr. */
    uint32_t txnCount(uint16_t addr) const;

    /** Total error transactions (status != MICROBIT_OK) for the device. */
    uint32_t errCount(uint16_t addr) const;

    /** Last non-OK CODAL status returned for the device. 0 if no errors. */
    int lastErr(uint16_t addr) const;

    // -----------------------------------------------------------------------
    // Re-entrancy diagnostics
    // -----------------------------------------------------------------------

    /** Total number of re-entrancy violations detected since construction
     *  (or last resetStats()). */
    uint32_t reentryViolations() const { return _reentryViolations; }

    /** 8-bit wire address that was in flight when the most recent violation
     *  was detected. */
    uint16_t reentryInFlightAddr() const { return _reentryInFlightAddr; }

    /** 8-bit wire address of the new caller that triggered the most recent
     *  violation. */
    uint16_t reentryNewAddr() const { return _reentryNewAddr; }

    // -----------------------------------------------------------------------
    // Utility
    // -----------------------------------------------------------------------

    /** Reset all counters and violation state to zero. */
    void resetStats();

private:
    // Maximum number of distinct devices tracked (including the "other" bucket
    // at index kMaxDevices-1).
    static constexpr int kMaxDevices = 8;

    struct DeviceSlot {
        uint16_t addr;       // 7-bit address (0 = empty slot)
        uint32_t txnCount;
        uint32_t errCount;
        int      lastErr;
    };

    MicroBitI2C& _bus;

    // Re-entrancy guard state.
    volatile bool _inUse;
    uint16_t      _inFlightAddr;  // wire address currently in flight (for guard)

    // Violation capture.
    uint32_t _reentryViolations;
    uint16_t _reentryInFlightAddr;
    uint16_t _reentryNewAddr;

    // Per-device slot table.
    DeviceSlot _devices[kMaxDevices];
    int        _deviceCount;

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /**
     * findOrAdd — return the slot index for the given 7-bit address.
     *
     * If the address already has a slot, that index is returned. If not, a
     * new slot is allocated (up to kMaxDevices-1). If the table is full, the
     * last slot (the "other" bucket) is returned so counters never overflow.
     */
    int findOrAdd(uint16_t addr7);

    /**
     * record — update per-device counters after a transaction completes.
     *
     * addr7:   7-bit device address.
     * status:  CODAL status int from the underlying MicroBitI2C call.
     */
    void record(uint16_t addr7, int status);
};
