#include "ColorSensor.h"

ColorSensor::ColorSensor(MicroBitI2C& i2c)
    : _i2c(i2c)
    , _isAlt(false)
    , _inited(false)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

bool ColorSensor::begin()
{
    // Probe the alt chip at 0x43 first.
    // Write reg 0x81 = 0xCA, reg 0x80 = 0x17 to wake up the alt chip, then
    // read reg 0xA4 (16-bit); a non-zero result confirms the alt chip.
    writeReg8(ADDR_ALT, 0x81, 0xCA);
    writeReg8(ADDR_ALT, 0x80, 0x17);
    uint16_t probe = readReg16(ADDR_ALT, 0xA4);
    if (probe != 0) {
        _isAlt = true;
        initAlt();
        _inited = true;
        return true;
    }

    // Fall back to APDS9960 at 0x39.
    // Perform a minimal communication check: write ENABLE register and read it back.
    // If the write succeeds we assume the chip is present (no device-ID register
    // on APDS9960 that is safe to probe without side effects at this stage).
    writeReg8(ADDR_APDS, 0x80, 0x00);  // ENABLE: power off — safe probe write
    uint8_t en = readReg8(ADDR_APDS, 0x80);
    if (en != 0x00) {
        // Unexpected read — device not present or not responding correctly.
        return false;
    }

    _isAlt = false;
    initApds();
    _inited = true;
    return true;
}

bool ColorSensor::readRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c)
{
    if (!_inited) return false;

    if (_isAlt) {
        // Alt chip: blocking 100 ms integration delay then read.
        fiber_sleep(100);
        c = readReg16(ADDR_ALT, 0xA6);
        r = readReg16(ADDR_ALT, 0xA0);
        g = readReg16(ADDR_ALT, 0xA2);
        b = readReg16(ADDR_ALT, 0xA4);
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
    // Alt chip is already probed and awake; no additional init required.
    // The probe sequence (write 0x81=0xCA, 0x80=0x17) has already started it.
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
