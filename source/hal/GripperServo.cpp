#include "GripperServo.h"

GripperServo::GripperServo(MicroBitPin& pin)
    : _pin(pin)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void GripperServo::setAngle(uint8_t degrees)
{
    if (degrees > 180) degrees = 180;
    _pin.setServoValue(degrees);
}
