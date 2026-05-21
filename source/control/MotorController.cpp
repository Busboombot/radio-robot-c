#include "MotorController.h"
#include <math.h>

MotorController::MotorController(NezhaV2& motor, const CalibParams& cal)
    : _motor(motor), _cal(cal),
      _pid(cal.ratioPidKp, cal.ratioPidKi, cal.ratioPidKd, cal.ratioPidMax),
      _cmdEncStartL(0.0f), _cmdEncStartR(0.0f),
      _cmdRatio(1.0f), _fasterIsRight(false),
      _tgtLMms(0.0f), _tgtRMms(0.0f),
      _prevEncL(0), _prevEncR(0),
      _actualVelL(0.0f), _actualVelR(0.0f)
{
    gains.kFF     = 0.15f;
    gains.kP      = 0.05f;
    gains.kI      = 0.20f;
    gains.iClamp  = 60.0f;
    gains.kRatio  = 0.01f;
}

float MotorController::encoderMm(bool left)
{
    return static_cast<float>(_motor.readEncoder(left, _cal));
}

void MotorController::setTarget(float leftMms, float rightMms)
{
    _tgtLMms = leftMms;
    _tgtRMms = rightMms;
}

void MotorController::startDriveClean(float leftMms, float rightMms)
{
    _tgtLMms = leftMms;
    _tgtRMms = rightMms;
    _fasterIsRight = (fabsf(rightMms) >= fabsf(leftMms));
    float fasterAbs = _fasterIsRight ? fabsf(rightMms) : fabsf(leftMms);
    float slowerAbs = _fasterIsRight ? fabsf(leftMms)  : fabsf(rightMms);
    _cmdRatio = (slowerAbs > 0.0f) ? (fasterAbs / slowerAbs) : 1.0f;
    _cmdEncStartL = encoderMm(true);
    _cmdEncStartR = encoderMm(false);
    _pid.reset();
}

void MotorController::startDrive(float leftMms, float rightMms)
{
    _tgtLMms = leftMms;
    _tgtRMms = rightMms;

    bool newFasterIsRight = (fabsf(rightMms) >= fabsf(leftMms));
    float newFasterAbs = newFasterIsRight ? fabsf(rightMms) : fabsf(leftMms);
    float newSlowerAbs = newFasterIsRight ? fabsf(leftMms)  : fabsf(rightMms);
    float newRatio = (newSlowerAbs > 0.0f) ? (newFasterAbs / newSlowerAbs) : 1.0f;

    float curL = encoderMm(true);
    float curR = encoderMm(false);
    float curFaster   = newFasterIsRight ? curR : curL;
    float curSlower   = newFasterIsRight ? curL : curR;
    float startFaster = newFasterIsRight ? _cmdEncStartR : _cmdEncStartL;
    float prevDeltaFaster = fabsf(curFaster - startFaster);

    float seedFaster = fmaxf(prevDeltaFaster, newFasterAbs);
    float seedSlower = (newRatio > 0.0f) ? (seedFaster / newRatio) : 0.0f;

    float signFaster = ((newFasterIsRight ? rightMms : leftMms) >= 0.0f) ? 1.0f : -1.0f;
    float signSlower = ((newFasterIsRight ? leftMms  : rightMms) >= 0.0f) ? 1.0f : -1.0f;

    if (newFasterIsRight) {
        _cmdEncStartR = curFaster - signFaster * seedFaster;
        _cmdEncStartL = curSlower - signSlower * seedSlower;
    } else {
        _cmdEncStartL = curFaster - signFaster * seedFaster;
        _cmdEncStartR = curSlower - signSlower * seedSlower;
    }

    if (newFasterIsRight != _fasterIsRight) _pid.reset();
    _fasterIsRight = newFasterIsRight;
    _cmdRatio = newRatio;
}

void MotorController::stop()
{
    _tgtLMms = 0.0f;
    _tgtRMms = 0.0f;
    _pid.reset();
    _cmdEncStartL = encoderMm(true);
    _cmdEncStartR = encoderMm(false);
    _motor.setPwm(0, 0);
}

void MotorController::resetIntegrators()
{
    _pid.reset();
}

void MotorController::updatePidGains(float kP, float kI, float kD, float iClamp)
{
    _pid.updateGains(kP, kI, kD, iClamp);
}

void MotorController::tick(float dt_s)
{
    if (dt_s <= 0.0f) return;

    // Step 1: Read encoder positions (mm)
    float encLMm = encoderMm(true);
    float encRMm = encoderMm(false);

    // Update velocity for getActualVelocity()
    _actualVelL = (encLMm - static_cast<float>(_prevEncL)) / dt_s;
    _actualVelR = (encRMm - static_cast<float>(_prevEncR)) / dt_s;
    _prevEncL = static_cast<int32_t>(encLMm);
    _prevEncR = static_cast<int32_t>(encRMm);

    // If no drive command active, ensure motors are stopped
    if (_tgtLMms == 0.0f && _tgtRMms == 0.0f) {
        _motor.setPwm(0, 0);
        return;
    }

    // Step 2: Cumulative deltas since command start
    float fDL = encLMm - _cmdEncStartL;
    float fDR = encRMm - _cmdEncStartR;
    float fasterDelta = _fasterIsRight ? fabsf(fDR) : fabsf(fDL);
    float slowerDelta  = _fasterIsRight ? fabsf(fDL) : fabsf(fDR);

    // Step 3: Normalized error
    float expected = slowerDelta * _cmdRatio;
    float normErr  = (expected - fasterDelta) / fmaxf(1.0f, expected);

    // Step 4: PID update
    float correction = _pid.update(normErr, dt_s);

    // Step 5: Feed-forward base PWM
    float scaleL = (_tgtLMms >= 0.0f) ? _cal.kScaleLF : _cal.kScaleLB;
    float scaleR = (_tgtRMms >= 0.0f) ? _cal.kScaleRF : _cal.kScaleRB;
    float tgtFasterAbs = _fasterIsRight ? fabsf(_tgtRMms) : fabsf(_tgtLMms);
    float tgtSlowerAbs = _fasterIsRight ? fabsf(_tgtLMms) : fabsf(_tgtRMms);
    float scaleFaster  = _fasterIsRight ? scaleR : scaleL;
    float scaleSlower  = _fasterIsRight ? scaleL : scaleR;
    float baseFaster = _cal.kFF * tgtFasterAbs * scaleFaster;
    float baseSlower = _cal.kFF * tgtSlowerAbs * scaleSlower;

    // Step 6: Slower-wheel adjustment
    float excess = _pid.integral - _cal.kAdjThreshold;
    float adj = (excess > 0.0f) ? (-_cal.kAdjGain * excess * baseFaster) : 0.0f;

    // Step 7: Compute and clamp final PWM
    float uFaster = clamp(baseFaster + correction, 0.0f, 100.0f);
    float uSlower = clamp(baseSlower + adj,        0.0f, 100.0f);

    // Apply direction signs
    float uL, uR;
    if (_fasterIsRight) {
        uL = (_tgtLMms >= 0.0f) ?  uSlower : -uSlower;
        uR = (_tgtRMms >= 0.0f) ?  uFaster : -uFaster;
    } else {
        uL = (_tgtLMms >= 0.0f) ?  uFaster : -uFaster;
        uR = (_tgtRMms >= 0.0f) ?  uSlower : -uSlower;
    }

    _motor.setPwm(static_cast<int8_t>(roundf(uL)), static_cast<int8_t>(roundf(uR)));
}

void MotorController::getActualVelocity(float& leftMms, float& rightMms) const
{
    leftMms  = _actualVelL;
    rightMms = _actualVelR;
}

void MotorController::getEncoderPositions(int32_t& leftMm, int32_t& rightMm) const
{
    leftMm  = _motor.readEncoder(true,  _cal);
    rightMm = _motor.readEncoder(false, _cal);
}

void MotorController::resetEncoderAccumulators()
{
    _motor.resetEncoders();
    _prevEncL = 0;
    _prevEncR = 0;
}

float MotorController::clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
