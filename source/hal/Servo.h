#pragma once
#include "MicroBit.h"
#include "IServo.h"
#include <stdint.h>

/**
 * Servo — CODAL pin driver for a hobby servo.
 *
 * Wraps MicroBitPin::setServoValue() to provide a clamped 0..maxDegrees
 * interface. Supports both standard 180° servos and 360° continuous-rotation
 * servos via the configurable maxDegrees parameter.
 */
class Servo : public IServo {
public:
    explicit Servo(MicroBitPin& pin, uint16_t maxDegrees = 180);

    // Set servo angle. Clamps to [0, maxDegrees] before driving the pin.
    // Records the clamped value; retrieve it with currentAngle().
    void setAngle(uint8_t degrees) override;

    // Return the last clamped angle passed to setAngle(). Defaults to 0 before
    // any setAngle() call.
    int16_t currentAngle() const override { return _currentAngle; }

private:
    MicroBitPin& _pin;
    uint16_t     _maxDegrees;
    int16_t      _currentAngle = 0;
};
