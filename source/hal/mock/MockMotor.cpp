#include "MockMotor.h"
#include "types/Config.h"

void MockMotor::setSpeed(int8_t pct) {
    _cmdSpeed = pct;
}

void MockMotor::requestEncoder() {
    // No-op: encoder is always ready in the mock.
}

int32_t MockMotor::collectEncoder() const {
    return static_cast<int32_t>(_encoderMm);
}

float MockMotor::readEncoderMmF(const RobotConfig& /*cfg*/) const {
    return _encoderMm;
}

float MockMotor::readEncoderMmFAtomic(const RobotConfig& /*cfg*/) const {
    return _encoderMm;
}

float MockMotor::readEncoderMmFSettle(const RobotConfig& /*cfg*/) const {
    return _encoderMm;
}

void MockMotor::resetEncoder() {
    _encoderMm = 0.0f;
    _cmdSpeed  = 0;
}

void MockMotor::tick(uint32_t dt_ms) {
    float vel = (_cmdSpeed / 100.0f) * kNominalMaxMms * _offsetFactor;
    _encoderMm += vel * (static_cast<float>(dt_ms) / 1000.0f);
}
