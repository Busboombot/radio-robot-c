#include "MockHAL.h"

void MockHAL::tick(uint32_t now_ms) {
    int32_t dt = static_cast<int32_t>(now_ms - _lastTickMs);
    if (dt > 0) {
        uint32_t udt = static_cast<uint32_t>(dt);
        _motorL.tick(udt);
        _motorR.tick(udt);
        _line.tick(udt);
        _color.tick(udt);
    }
    _lastTickMs = now_ms;
}
