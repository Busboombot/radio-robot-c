#pragma once
#include "MicroBit.h"
#include <stdint.h>

/**
 * LineSensor — I2C driver for the PlanetX line sensor.
 *
 * I2C address: 0x1A (7-bit).
 *
 * Protocol: write 1-byte channel index (0-3), then read 1 byte of grayscale
 * data (0 = white, 255 = black approximately).
 */
class LineSensor {
public:
    explicit LineSensor(MicroBitI2C& i2c);

    // Fills out[0..3] with grayscale values (0=white, 255=black approx).
    // Returns false on I2C error. out may be nullptr (probe use).
    bool readValues(uint16_t out[4]) const;

private:
    MicroBitI2C& _i2c;
    static constexpr uint8_t ADDR = 0x1A;
};
