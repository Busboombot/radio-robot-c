#include "I2CBus.h"
#include "codal_target_hal.h"   // target_disable_irq() / target_enable_irq()

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

I2CBus::I2CBus(MicroBitI2C& bus)
    : _bus(bus)
    , _inUse(false)
    , _inFlightAddr(0)
    , _reentryViolations(0)
    , _reentryInFlightAddr(0)
    , _reentryNewAddr(0)
    , _deviceCount(0)
{
    for (int i = 0; i < kMaxDevices; ++i) {
        _devices[i].addr     = 0;
        _devices[i].txnCount = 0;
        _devices[i].errCount = 0;
        _devices[i].lastErr  = 0;
    }
}

// ---------------------------------------------------------------------------
// I2C forwarding
// ---------------------------------------------------------------------------

int I2CBus::write(uint16_t address, uint8_t* data, int len, bool repeated)
{
    // address is the 8-bit wire address (7-bit addr << 1).
    uint16_t addr7 = (uint16_t)(address >> 1);

    // --- Re-entrancy guard: atomic check-and-set ---
    // Disable IRQs for the MINIMUM window needed to check+set the flag.
    // The full I2C transaction is NOT inside the critical section — only
    // the 3-instruction flag read-modify-write is.
    target_disable_irq();
    bool alreadyInUse = _inUse;
    if (alreadyInUse) {
        // Capture violation: keep the existing _inFlightAddr, record new addr.
        ++_reentryViolations;
        _reentryInFlightAddr = _inFlightAddr;
        _reentryNewAddr      = (uint16_t)address;
    } else {
        _inUse        = true;
        _inFlightAddr = (uint16_t)address;
    }
    target_enable_irq();
    // --- End critical section ---

    int status = _bus.write(address, data, len, repeated);

    // Clear the flag only if we were the one who set it.
    if (!alreadyInUse) {
        _inUse = false;
    }

    record(addr7, status);
    return status;
}

int I2CBus::read(uint16_t address, uint8_t* data, int len, bool repeated)
{
    uint16_t addr7 = (uint16_t)(address >> 1);

    // --- Re-entrancy guard: atomic check-and-set ---
    target_disable_irq();
    bool alreadyInUse = _inUse;
    if (alreadyInUse) {
        ++_reentryViolations;
        _reentryInFlightAddr = _inFlightAddr;
        _reentryNewAddr      = (uint16_t)address;
    } else {
        _inUse        = true;
        _inFlightAddr = (uint16_t)address;
    }
    target_enable_irq();
    // --- End critical section ---

    int status = _bus.read(address, data, len, repeated);

    if (!alreadyInUse) {
        _inUse = false;
    }

    record(addr7, status);
    return status;
}

// ---------------------------------------------------------------------------
// Per-device statistics
// ---------------------------------------------------------------------------

uint32_t I2CBus::txnCount(uint16_t addr) const
{
    for (int i = 0; i < _deviceCount; ++i) {
        if (_devices[i].addr == addr) return _devices[i].txnCount;
    }
    return 0;
}

uint32_t I2CBus::errCount(uint16_t addr) const
{
    for (int i = 0; i < _deviceCount; ++i) {
        if (_devices[i].addr == addr) return _devices[i].errCount;
    }
    return 0;
}

int I2CBus::lastErr(uint16_t addr) const
{
    for (int i = 0; i < _deviceCount; ++i) {
        if (_devices[i].addr == addr) return _devices[i].lastErr;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------

void I2CBus::resetStats()
{
    _reentryViolations   = 0;
    _reentryInFlightAddr = 0;
    _reentryNewAddr      = 0;
    _inUse               = false;
    _inFlightAddr        = 0;

    for (int i = 0; i < kMaxDevices; ++i) {
        _devices[i].addr     = 0;
        _devices[i].txnCount = 0;
        _devices[i].errCount = 0;
        _devices[i].lastErr  = 0;
    }
    _deviceCount = 0;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

int I2CBus::findOrAdd(uint16_t addr7)
{
    // Linear scan: return existing slot if found.
    for (int i = 0; i < _deviceCount; ++i) {
        if (_devices[i].addr == addr7) return i;
    }

    // Allocate a new slot if there is room (leave the last slot as
    // the "other" bucket so we never exceed the array bounds).
    if (_deviceCount < kMaxDevices - 1) {
        int idx = _deviceCount++;
        _devices[idx].addr     = addr7;
        _devices[idx].txnCount = 0;
        _devices[idx].errCount = 0;
        _devices[idx].lastErr  = 0;
        return idx;
    }

    // Table full: use the last slot as the "other" overflow bucket.
    // Mark it addr=0xFFFF so queries for unknown addresses don't
    // collide with real devices (real 7-bit addrs are 0x00..0x7F).
    int overflow = kMaxDevices - 1;
    if (_devices[overflow].addr == 0) {
        _devices[overflow].addr = 0xFFFF;
        _deviceCount = kMaxDevices;
    }
    return overflow;
}

void I2CBus::record(uint16_t addr7, int status)
{
    int idx = findOrAdd(addr7);
    ++_devices[idx].txnCount;
    if (status != MICROBIT_OK) {
        ++_devices[idx].errCount;
        _devices[idx].lastErr = status;
    }
}
