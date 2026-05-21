#include "LineSensor.h"

LineSensor::LineSensor(MicroBitI2C& i2c)
    : _i2c(i2c)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

bool LineSensor::readValues(uint16_t out[4]) const
{
    for (uint8_t ch = 0; ch < 4; ch++) {
        // Write the channel index byte.
        uint8_t chByte = ch;
        int rc = _i2c.write((uint16_t)(ADDR << 1), (uint8_t*)&chByte, 1, false);
        if (rc != MICROBIT_OK) return false;

        // Read 1 byte of grayscale data.
        uint8_t val = 0;
        rc = _i2c.read((uint16_t)(ADDR << 1), (uint8_t*)&val, 1, false);
        if (rc != MICROBIT_OK) return false;

        if (out) out[ch] = val;
    }
    return true;
}
