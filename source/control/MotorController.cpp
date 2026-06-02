#include "MotorController.h"
#include <math.h>

MotorController::MotorController(Motor& left, Motor& right, const RobotConfig& cal)
    : _motorL(left), _motorR(right), _cal(cal),
      _pid(cal.ratioPidKp, cal.ratioPidKi, cal.ratioPidKd, cal.ratioPidMax),
      _cmdEncStartL(0.0f), _cmdEncStartR(0.0f),
      _cmdRatio(1.0f), _fasterIsRight(false),
      _tgtLMms(0.0f), _tgtRMms(0.0f),
      _prevEncL(0), _prevEncR(0),
      _actualVelL(0.0f), _actualVelR(0.0f),
      _encLMm(0.0f), _encRMm(0.0f),
      _usingChipVelL(false), _usingChipVelR(false)
{
    gains.kFF     = 0.15f;
    gains.kP      = 0.05f;
    gains.kI      = 0.20f;
    gains.iClamp  = 60.0f;
    gains.kRatio  = 0.01f;
}

float MotorController::encoderMm(bool left)
{
    return static_cast<float>(left ? _motorL.readEncoder(_cal) : _motorR.readEncoder(_cal));
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
    _motorL.setSpeed(0);
    _motorR.setSpeed(0);
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

    // Step 1: Read encoder positions (mm) and cache for getEncoderPositions()
    float encLMm = encoderMm(true);
    float encRMm = encoderMm(false);
    _encLMm = encLMm;
    _encRMm = encRMm;

    // Encoder-delta velocity (fallback / implausibility reference)
    float encVelL = (encLMm - static_cast<float>(_prevEncL)) / dt_s;
    float encVelR = (encRMm - static_cast<float>(_prevEncR)) / dt_s;
    _prevEncL = static_cast<int32_t>(encLMm);
    _prevEncR = static_cast<int32_t>(encRMm);

    // Chip-native velocity (primary source via register 0x47).
    // Falls back to encoder-delta if:
    //   (a) I2C read fails (readSpeed returns false), or
    //   (b) chip reading exceeds 2× the encoder-derived velocity (implausibility gate).
    float chipVelL = 0.0f, chipVelR = 0.0f;
    bool chipOkL = _motorL.readSpeed(chipVelL, _cal);
    bool chipOkR = _motorR.readSpeed(chipVelR, _cal);

    // Implausibility gate: reject chip reading if it is more than 2× encoder velocity.
    // This guards against I2C noise producing out-of-range readings.
    if (chipOkL && fabsf(encVelL) > 0.0f &&
        fabsf(chipVelL) > 2.0f * fabsf(encVelL)) {
        chipOkL = false;
    }
    if (chipOkR && fabsf(encVelR) > 0.0f &&
        fabsf(chipVelR) > 2.0f * fabsf(encVelR)) {
        chipOkR = false;
    }

    _usingChipVelL = chipOkL;
    _usingChipVelR = chipOkR;
    _actualVelL = chipOkL ? chipVelL : encVelL;
    _actualVelR = chipOkR ? chipVelR : encVelR;

    // If no drive command active, ensure motors are stopped
    if (_tgtLMms == 0.0f && _tgtRMms == 0.0f) {
        _motorL.setSpeed(0);
        _motorR.setSpeed(0);
        return;
    }

    // Step 2: Cumulative deltas since command start
    float fDL = encLMm - _cmdEncStartL;
    float fDR = encRMm - _cmdEncStartR;
    float fasterDelta = _fasterIsRight ? fabsf(fDR) : fabsf(fDL);
    float slowerDelta  = _fasterIsRight ? fabsf(fDL) : fabsf(fDR);

    // Step 3: Normalized error.
    float expected     = slowerDelta * _cmdRatio;
    float tgtSlowerAbs = _fasterIsRight ? fabsf(_tgtLMms) : fabsf(_tgtRMms);
    float tgtFasterAbs = _fasterIsRight ? fabsf(_tgtRMms) : fabsf(_tgtLMms);
    float denomFloor   = fmaxf(tgtFasterAbs, 1.0f);
    float normErr      = (expected - fasterDelta) / fmaxf(denomFloor, expected);

    // Step 4: PID update
    float correction = _pid.update(normErr, dt_s);

    // Step 5: Feed-forward base PWM
    float scaleL = (_tgtLMms >= 0.0f) ? _cal.kScaleLF : _cal.kScaleLB;
    float scaleR = (_tgtRMms >= 0.0f) ? _cal.kScaleRF : _cal.kScaleRB;
    float scaleFaster  = _fasterIsRight ? scaleR : scaleL;
    float scaleSlower  = _fasterIsRight ? scaleL : scaleR;
    float baseFaster = _cal.kFF * tgtFasterAbs * scaleFaster;
    float baseSlower = _cal.kFF * tgtSlowerAbs * scaleSlower;

    // Step 6: Slower-wheel adjustment
    float excess = _pid.integral - _cal.kAdjThreshold;
    float adj = (excess > 0.0f) ? (-_cal.kAdjGain * excess * baseFaster) : 0.0f;

    // Step 7: Compute and clamp final PWM
    float rawFaster = baseFaster + correction;
    float uFaster = clamp(rawFaster, 0.0f, 100.0f);
    // When faster wheel is pegged at floor, redirect overflow to boost slower wheel.
    float fasterOverflow = fminf(rawFaster, 0.0f);
    float adjBoost = -fasterOverflow
                     * (baseSlower / fmaxf(baseFaster, 1.0f))
                     / fmaxf(_cmdRatio, 1.0f);
    // Proportional slower-wheel boost: boosts slower when faster over-runs.
    // Drives normErr toward 0 from the slower side; gain tuned so equilibrium is at ratio≈cmdRatio.
    float propBoost = (normErr < 0.0f) ? (-3.0f * normErr * baseSlower) : 0.0f;
    float uSlower = clamp(baseSlower + adj + adjBoost + propBoost, 0.0f, 100.0f);

    // Apply direction signs
    float uL, uR;
    if (_fasterIsRight) {
        uL = (_tgtLMms >= 0.0f) ?  uSlower : -uSlower;
        uR = (_tgtRMms >= 0.0f) ?  uFaster : -uFaster;
    } else {
        uL = (_tgtLMms >= 0.0f) ?  uFaster : -uFaster;
        uR = (_tgtRMms >= 0.0f) ?  uSlower : -uSlower;
    }

    _motorL.setSpeed(static_cast<int8_t>(roundf(uL)));
    _motorR.setSpeed(static_cast<int8_t>(roundf(uR)));
}

void MotorController::getActualVelocity(float& leftMms, float& rightMms) const
{
    leftMms  = _actualVelL;
    rightMms = _actualVelR;
}

void MotorController::getVelocitySourceFlags(bool& leftChip, bool& rightChip) const
{
    leftChip  = _usingChipVelL;
    rightChip = _usingChipVelR;
}

void MotorController::getEncoderPositions(int32_t& leftMm, int32_t& rightMm) const
{
    leftMm  = static_cast<int32_t>(_encLMm);
    rightMm = static_cast<int32_t>(_encRMm);
}

void MotorController::resetEncoderAccumulators()
{
    _motorL.resetEncoder();
    _motorR.resetEncoder();
    _prevEncL = 0;
    _prevEncR = 0;
}

float MotorController::clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
