#include "PortIO.h"

PortIO::PortIO(MicroBitIO& io)
    : _io(io)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void PortIO::setDigital(uint8_t port, bool high)
{
    MicroBitPin* pin = digitalPin(port);
    if (!pin) return;
    pin->setDigitalValue(high ? 1 : 0);
}

int PortIO::readDigital(uint8_t port) const
{
    MicroBitPin* pin = digitalPin(port);
    if (!pin) return -1;
    return pin->getDigitalValue();
}

void PortIO::setAnalog(uint8_t port, uint16_t val)
{
    MicroBitPin* pin = analogPin(port);
    if (!pin) return;
    pin->setAnalogValue(val);
}

int PortIO::readAnalog(uint8_t port) const
{
    MicroBitPin* pin = analogPin(port);
    if (!pin) return -1;
    return pin->getAnalogValue();
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

MicroBitPin* PortIO::digitalPin(uint8_t port) const
{
    switch (port) {
        case 1: return &_io.P8;
        case 2: return &_io.P12;
        case 3: return &_io.P14;
        case 4: return &_io.P16;
        default: return nullptr;
    }
}

MicroBitPin* PortIO::analogPin(uint8_t port) const
{
    switch (port) {
        case 1: return &_io.P1;
        case 2: return &_io.P2;
        case 3: return &_io.P13;
        case 4: return &_io.P15;
        default: return nullptr;
    }
}
