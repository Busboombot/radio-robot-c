#pragma once
#include "MicroBit.h"
#include "I2CBus.h"
#include "Sensor.h"
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
class ColorSensor : public Sensor {
public:
    explicit ColorSensor(I2CBus& i2c);

    // Auto-detect chip variant. Returns false if neither 0x43 nor 0x39 responds.
    // Sets _initialized to the result.
    bool begin() override;

    // Fills r, g, b, c with 16-bit raw counts. Blocks up to ~250 ms.
    bool readRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c);

    // Non-blocking poll: returns true and fills outputs only if fresh data is
    // available immediately. Returns false (does not block) if sensor is not ready.
    // Use this in time-critical loops instead of readRGBC().
    bool pollRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c);

private:
    I2CBus& _i2c;
    bool _isAlt;

    static constexpr uint8_t ADDR_APDS = 0x39;
    static constexpr uint8_t ADDR_ALT  = 0x43;

    void initApds();
    void initAlt();

    void     writeReg8(uint8_t addr, uint8_t reg, uint8_t val);
    uint8_t  readReg8(uint8_t addr, uint8_t reg) const;
    uint16_t readReg16(uint8_t addr, uint8_t regLo) const;
    uint16_t readReg16Alt(uint8_t regLo) const;  // alt-chip single-byte protocol
};
