#include "MockServo.h"

void MockServo::setAngle(uint8_t degrees) {
    _angle = static_cast<int16_t>(degrees);
}

int16_t MockServo::currentAngle() const {
    return _angle;
}
