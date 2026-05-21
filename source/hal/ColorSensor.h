#pragma once
#include "MicroBit.h"
#include <stdint.h>

/**
 * ColorSensor — I2C driver for RGBC color sensing.
 *
 * Supports two chip variants:
 *   - Alt chip at 0x43 (PlanetX / Elecfreaks variant)
 *   - APDS9960 at 0x39 (fallback)
 *
 * begin() probes 0x43 first; if the probe fails, initialises APDS9960 at 0x39.
 */
class ColorSensor {
public:
    explicit ColorSensor(MicroBitI2C& i2c);

    // Auto-detect chip variant. Returns false if neither 0x43 nor 0x39 responds.
    bool begin();

    // Fills r, g, b, c with 16-bit raw counts. Blocks up to ~250 ms.
    bool readRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c);

private:
    MicroBitI2C& _i2c;
    bool _isAlt;
    bool _inited;

    static constexpr uint8_t ADDR_APDS = 0x39;
    static constexpr uint8_t ADDR_ALT  = 0x43;

    void initApds();
    void initAlt();

    void     writeReg8(uint8_t addr, uint8_t reg, uint8_t val);
    uint8_t  readReg8(uint8_t addr, uint8_t reg) const;
    uint16_t readReg16(uint8_t addr, uint8_t regLo) const;
};
