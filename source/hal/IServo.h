#pragma once
#include <stdint.h>

/**
 * IServo — pure-virtual interface for a hobby servo.
 */
class IServo {
public:
    virtual ~IServo() = default;

    // Set servo angle, clamped to the servo's configured maximum.
    virtual void setAngle(uint8_t degrees) = 0;

    // Return the last clamped angle passed to setAngle(). Defaults to 0 before
    // any setAngle() call.
    virtual int16_t currentAngle() const = 0;
};
