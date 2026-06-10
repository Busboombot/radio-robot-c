#pragma once
#include <stdint.h>
#include "../IPortIO.h"

/**
 * MockPortIO — host-compilable IPortIO implementation for unit tests.
 *
 * Stores digital and analog state per port (1..4). Reads return last-written
 * value.  Out-of-range port: setDigital/setAnalog are no-ops; read returns -1.
 */
class MockPortIO : public IPortIO {
public:
    static constexpr uint8_t kMinPort = 1;
    static constexpr uint8_t kMaxPort = 4;

    // IPortIO interface ------------------------------------------------------
    void setDigital(uint8_t port, bool high) override;
    int  readDigital(uint8_t port) const override;
    void setAnalog(uint8_t port, uint16_t val) override;
    int  readAnalog(uint8_t port) const override;

private:
    // Indexed 0..3 for ports 1..4.
    bool     _digital[4] = {false, false, false, false};
    uint16_t _analog[4]  = {0, 0, 0, 0};

    bool valid(uint8_t port) const {
        return port >= kMinPort && port <= kMaxPort;
    }
    int idx(uint8_t port) const { return static_cast<int>(port) - 1; }
};
