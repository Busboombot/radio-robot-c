#include "RatioPidController.h"

RatioPidController::RatioPidController(float kP, float kI, float kD, float iClamp)
    : _kP(kP), _kI(kI), _kD(kD), _iClamp(iClamp), _prevError(0.0f), _firstCall(true), integral(0.0f) {}

float RatioPidController::update(float error, float dtS) {
    integral += _kI * error * dtS;
    if (integral > _iClamp) integral = _iClamp;
    if (integral < -_iClamp) integral = -_iClamp;
    float deriv = 0.0f;
    if (!_firstCall && dtS > 0.0f) {
        deriv = (error - _prevError) / dtS;
    }
    _firstCall = false;
    _prevError = error;
    return _kP * error + integral + _kD * deriv;
}

void RatioPidController::reset() {
    integral = 0.0f;
    _prevError = 0.0f;
    _firstCall = true;
}

float RatioPidController::clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
