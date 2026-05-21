#pragma once
#include "MicroBit.h"
#include <stdint.h>

/**
 * PortIO — CODAL IO abstraction for the four sensor/actuator ports on the
 * PlanetX Nezha V2 expansion board.
 *
 * Pin mapping:
 *   Digital S2 connector: J1→P8,  J2→P12, J3→P14, J4→P16
 *   Analog  S1 connector: J1→P1,  J2→P2,  J3→P13, J4→P15
 *
 * Port numbers are 1-based (1..4); out-of-range values return -1 or are ignored.
 */
class PortIO {
public:
    explicit PortIO(MicroBitIO& io);

    // Set the digital output on port 1..4. Ignores out-of-range port.
    void setDigital(uint8_t port, bool high);

    // Read the digital input on port 1..4. Returns 0 or 1, or -1 for invalid port.
    int  readDigital(uint8_t port) const;

    // Set the analog (PWM) output on port 1..4, value 0..1023.
    void setAnalog(uint8_t port, uint16_t val);

    // Read the analog input on port 1..4, value 0..1023. Returns -1 for invalid port.
    int  readAnalog(uint8_t port) const;

private:
    MicroBitIO& _io;

    // Returns the digital pin for port 1..4, or nullptr for out-of-range.
    MicroBitPin* digitalPin(uint8_t port) const;

    // Returns the analog pin for port 1..4, or nullptr for out-of-range.
    MicroBitPin* analogPin(uint8_t port) const;
};
