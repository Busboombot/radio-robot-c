#include "ColorSensor.h"

ColorSensor::ColorSensor(MicroBitI2C& i2c)
    : _i2c(i2c)
    , _isAlt(false)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

bool ColorSensor::begin()
{
    // EXACT port of upstream PlanetX initColor (old radio-robot nezha.ts):
    // probe the alt chip at 0x43, re-asserting the wake writes (0x81=0xCA,
    // 0x80=0x17) INSIDE every retry iteration, settle 50 ms, then check the
    // 16-bit value at 0xA4/0xA5 is non-zero.  Re-writing the wake each loop is
    // what wakes a chip that wasn't ready on the first attempt.
    for (int i = 0; i < 20; i++) {
        writeReg8(ADDR_ALT, 0x81, 0xCA);
        writeReg8(ADDR_ALT, 0x80, 0x17);
        fiber_sleep(50);
        if (readReg16Alt(0xA4) != 0) {   // 0xA4 + 0xA5 * 256
            _isAlt = true;
            _initialized = true;
            return true;
        }
    }

    // Fall back to APDS9960 at 0x39: write ENABLE off and read it back.
    writeReg8(ADDR_APDS, 0x80, 0x00);
    if (readReg8(ADDR_APDS, 0x80) == 0x00) {
        _isAlt = false;
        initApds();
        _initialized = true;
        return true;
    }
    return false;
}

bool ColorSensor::pollRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c)
{
    if (!is_initialized()) return false;
    if (_isAlt) {
        // Alt chip: check if data ready by reading clear channel; non-zero = ready.
        // Use two single-byte reads per 16-bit value, mirroring upstream protocol.
        uint16_t probe = readReg16Alt(0xA6);
        if (probe == 0) return false;
        c = probe;
        r = readReg16Alt(0xA0);
        g = readReg16Alt(0xA2);
        b = readReg16Alt(0xA4);
    } else {
        // APDS9960: check AVALID bit without blocking.
        if ((readReg8(ADDR_APDS, 0x93) & 0x01) == 0) return false;
        c = readReg16(ADDR_APDS, 0x94);
        r = readReg16(ADDR_APDS, 0x96);
        g = readReg16(ADDR_APDS, 0x98);
        b = readReg16(ADDR_APDS, 0x9A);
    }
    return true;
}

bool ColorSensor::readRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c)
{
    if (!is_initialized()) return false;

    if (_isAlt) {
        // Alt chip: blocking 100 ms integration delay then read.
        // Use two single-byte reads per 16-bit value, mirroring upstream protocol.
        fiber_sleep(100);
        c = readReg16Alt(0xA6);
        r = readReg16Alt(0xA0);
        g = readReg16Alt(0xA2);
        b = readReg16Alt(0xA4);
    } else {
        // APDS9960: poll STATUS register bit 0 (AVALID), max 50 × 5 ms = 250 ms.
        int tries = 0;
        while (((readReg8(ADDR_APDS, 0x93) & 0x01) == 0) && tries < 50) {
            fiber_sleep(5);
            tries++;
        }
        c = readReg16(ADDR_APDS, 0x94);
        r = readReg16(ADDR_APDS, 0x96);
        g = readReg16(ADDR_APDS, 0x98);
        b = readReg16(ADDR_APDS, 0x9A);
    }
    return true;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void ColorSensor::initAlt()
{
    // Init is performed inline in begin() (write 0x81=0xCA, 0x80=0x17 + settle/retry).
    // Nothing additional needed here.
}

void ColorSensor::initApds()
{
    writeReg8(ADDR_APDS, 0x81, 252);   // ATIME: integration time
    writeReg8(ADDR_APDS, 0x8F, 0x03);  // CONTROL: 4x gain
    writeReg8(ADDR_APDS, 0x80, 0x00);  // ENABLE: power off
    writeReg8(ADDR_APDS, 0xAB, 0x00);
    writeReg8(ADDR_APDS, 0xE7, 0x00);
    writeReg8(ADDR_APDS, 0x80, 0x01);  // ENABLE: power on
    // Enable AEN (ambient/colour enable) bit.
    uint8_t en = readReg8(ADDR_APDS, 0x80);
    writeReg8(ADDR_APDS, 0x80, en | 0x02);
}

void ColorSensor::writeReg8(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    _i2c.write((addr << 1), (uint8_t*)buf, 2, false);
}

uint8_t ColorSensor::readReg8(uint8_t addr, uint8_t reg) const
{
    uint8_t result = 0;
    _i2c.write((addr << 1), (uint8_t*)&reg, 1, false);
    _i2c.read((addr << 1), (uint8_t*)&result, 1, false);
    return result;
}

uint16_t ColorSensor::readReg16(uint8_t addr, uint8_t regLo) const
{
    // Read two consecutive bytes: [regLo, regLo+1] → little-endian uint16.
    uint8_t raw[2] = {0, 0};
    _i2c.write((addr << 1), (uint8_t*)&regLo, 1, false);
    _i2c.read((addr << 1), (uint8_t*)raw, 2, false);
    return (uint16_t)(raw[0] | ((uint16_t)raw[1] << 8));
}

uint16_t ColorSensor::readReg16Alt(uint8_t regLo) const
{
    // Alt-chip (0x43) single-byte protocol: issue two separate write-reg/read-1-byte
    // transactions instead of one 2-byte burst read.  Mirrors the upstream PlanetX
    // driver exactly (i2cread_color lo + i2cread_color hi * 256).
    uint8_t lo = readReg8(ADDR_ALT, regLo);
    uint8_t hi = readReg8(ADDR_ALT, (uint8_t)(regLo + 1));
    return (uint16_t)(lo | ((uint16_t)hi << 8));
}
