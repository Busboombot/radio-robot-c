#include "MockPortIO.h"

void MockPortIO::setDigital(uint8_t port, bool high) {
    if (!valid(port)) return;
    _digital[idx(port)] = high;
}

int MockPortIO::readDigital(uint8_t port) const {
    if (!valid(port)) return -1;
    return _digital[idx(port)] ? 1 : 0;
}

void MockPortIO::setAnalog(uint8_t port, uint16_t val) {
    if (!valid(port)) return;
    _analog[idx(port)] = val;
}

int MockPortIO::readAnalog(uint8_t port) const {
    if (!valid(port)) return -1;
    return static_cast<int>(_analog[idx(port)]);
}
