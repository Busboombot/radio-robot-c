#pragma once
#include <stdint.h>
#include "../IServo.h"

/**
 * MockServo — host-compilable IServo implementation for unit tests.
 *
 * Records the last angle passed to setAngle(). currentAngle() returns the
 * stored value. Defaults to 0 before any setAngle() call.
 */
class MockServo : public IServo {
public:
    // IServo interface -------------------------------------------------------
    void    setAngle(uint8_t degrees) override;
    int16_t currentAngle() const override;

private:
    int16_t _angle = 0;
};
