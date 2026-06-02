#pragma once
#include "MicroBit.h"
#include <stdint.h>

/**
 * Servo — CODAL pin driver for a hobby servo.
 *
 * Wraps MicroBitPin::setServoValue() to provide a clamped 0..maxDegrees
 * interface. Supports both standard 180° servos and 360° continuous-rotation
 * servos via the configurable maxDegrees parameter.
 */
class Servo {
public:
    explicit Servo(MicroBitPin& pin, uint16_t maxDegrees = 180);

    // Set servo angle. Clamps to [0, maxDegrees] before driving the pin.
    void setAngle(uint8_t degrees);

private:
    MicroBitPin& _pin;
    uint16_t     _maxDegrees;
};
