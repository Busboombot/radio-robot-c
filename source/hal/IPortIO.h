#pragma once
#include <stdint.h>

/**
 * IPortIO — pure-virtual interface for the four sensor/actuator port pins.
 */
class IPortIO {
public:
    virtual ~IPortIO() = default;

    // Set the digital output on port 1..4. Ignores out-of-range port.
    virtual void setDigital(uint8_t port, bool high) = 0;

    // Read the digital input on port 1..4. Returns 0 or 1, or -1 for invalid.
    virtual int readDigital(uint8_t port) const = 0;

    // Set the analog (PWM) output on port 1..4, value 0..1023.
    virtual void setAnalog(uint8_t port, uint16_t val) = 0;

    // Read the analog input on port 1..4, value 0..1023. Returns -1 for invalid.
    virtual int readAnalog(uint8_t port) const = 0;
};
