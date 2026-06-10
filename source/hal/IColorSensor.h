#pragma once
#include <stdint.h>
#include "Sensor.h"

/**
 * IColorSensor — interface for an RGBC color sensor.
 *
 * Extends Sensor so that begin() and is_initialized() are provided by the
 * Sensor base.
 */
class IColorSensor : public Sensor {
public:
    virtual ~IColorSensor() = default;

    // Fills r, g, b, c with 16-bit raw counts. Blocks up to ~250 ms.
    virtual bool readRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) = 0;

    // Non-blocking poll: returns true and fills outputs only if fresh data is
    // available immediately. Returns false (does not block) if not ready.
    virtual bool pollRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) = 0;
};
