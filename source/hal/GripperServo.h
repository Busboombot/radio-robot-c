#pragma once
#include "MicroBit.h"
#include <stdint.h>

/**
 * GripperServo — CODAL pin driver for a hobby servo gripper.
 *
 * Wraps MicroBitPin::setServoValue() to provide a clamped 0..180 degree interface.
 */
class GripperServo {
public:
    explicit GripperServo(MicroBitPin& pin);

    // Set servo angle. Clamps to 0..180 before driving the pin.
    void setAngle(uint8_t degrees);

private:
    MicroBitPin& _pin;
};
