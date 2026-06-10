#pragma once
#include <stdint.h>
#include "Sensor.h"

/**
 * ILineSensor — interface for a 4-channel line sensor.
 *
 * Extends Sensor so that begin() and is_initialized() are provided by the
 * Sensor base.
 */
class ILineSensor : public Sensor {
public:
    virtual ~ILineSensor() = default;

    // Fills out[0..3] with raw grayscale values. Returns false on I2C error.
    virtual bool readValues(uint16_t out[4]) const = 0;

    // Fills out[0..3] with normalized values [0, 1000]. Returns false on error.
    virtual bool readNormalized(uint16_t out[4]) = 0;
};
