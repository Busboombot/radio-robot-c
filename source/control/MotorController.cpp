#include "MotorController.h"
#include <math.h>

MotorController::MotorController(Motor& left, Motor& right, const RobotConfig& cal)
    : _motorL(left), _motorR(right), _cal(cal),
      _vcL(cal.velKff, cal.velKp, cal.velKi, 60.0f, cal.minWheelMms),
      _vcR(cal.velKff, cal.velKp, cal.velKi, 60.0f, cal.minWheelMms),
      _pid(cal.ratioPidKp, cal.ratioPidKi, cal.ratioPidKd, cal.ratioPidMax),
      _cmdEncStartL(0.0f), _cmdEncStartR(0.0f),
      _cmdRatio(1.0f), _fasterIsRight(false),
      _cmds(nullptr),
      _prevEncL(0.0f), _prevEncR(0.0f),
      _prevTimeMsL(0), _prevTimeMsR(0),
      _hasTimestampL(false), _hasTimestampR(false)
{
    gains.kFF     = 0.15f;
    gains.kP      = 0.05f;
    gains.kI      = 0.20f;
    gains.iClamp  = 60.0f;
    gains.kRatio  = 0.01f;
}

void MotorController::setTarget(float leftMms, float rightMms)
{
    if (_cmds) {
        _cmds->tgtLMms = leftMms;
        _cmds->tgtRMms = rightMms;
    }
}

void MotorController::startDriveClean(float leftMms, float rightMms)
{
    if (_cmds) {
        _cmds->tgtLMms = leftMms;
        _cmds->tgtRMms = rightMms;
    }
    _fasterIsRight = (fabsf(rightMms) >= fabsf(leftMms));
    float fasterAbs = _fasterIsRight ? fabsf(rightMms) : fabsf(leftMms);
    float slowerAbs = _fasterIsRight ? fabsf(leftMms)  : fabsf(rightMms);
    _cmdRatio = (slowerAbs > 0.0f) ? (fasterAbs / slowerAbs) : 1.0f;
    // Use atomic reads (request → 4 ms wait → collect) so the snapshots are
    // valid when called outside the split-phase control tick.
    _cmdEncStartL = _motorL.readEncoderMmFAtomic(_cal);
    _cmdEncStartR = _motorR.readEncoderMmFAtomic(_cal);
    _pid.reset();
    _vcL.reset();
    _vcR.reset();
}

void MotorController::startDrive(float leftMms, float rightMms)
{
    if (_cmds) {
        _cmds->tgtLMms = leftMms;
        _cmds->tgtRMms = rightMms;
    }

    bool newFasterIsRight = (fabsf(rightMms) >= fabsf(leftMms));
    float newFasterAbs = newFasterIsRight ? fabsf(rightMms) : fabsf(leftMms);
    float newSlowerAbs = newFasterIsRight ? fabsf(leftMms)  : fabsf(rightMms);
    float newRatio = (newSlowerAbs > 0.0f) ? (newFasterAbs / newSlowerAbs) : 1.0f;

    // Use atomic reads for valid position snapshots outside the control tick.
    float curL = _motorL.readEncoderMmFAtomic(_cal);
    float curR = _motorR.readEncoderMmFAtomic(_cal);
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
    if (_cmds) {
        _cmds->tgtLMms = 0.0f;
        _cmds->tgtRMms = 0.0f;
    }
    _pid.reset();
    _vcL.reset();
    _vcR.reset();
    // Use atomic reads for valid position snapshots outside the control tick.
    _cmdEncStartL = _motorL.readEncoderMmFAtomic(_cal);
    _cmdEncStartR = _motorR.readEncoderMmFAtomic(_cal);
    _motorL.setSpeed(0);
    _motorR.setSpeed(0);
}

void MotorController::resetIntegrators()
{
    _pid.reset();
    _vcL.reset();
    _vcR.reset();
}

void MotorController::updatePidGains(float kP, float kI, float kD, float iClamp)
{
    _pid.updateGains(kP, kI, kD, iClamp);
}

void MotorController::controlTick(HardwareState& inputs, MotorCommands& cmds,
                                    uint32_t now_ms, int refreshedWheel)
{
    // Per-wheel zero-order-hold velocity update (014-007 ZOH fix).
    //
    // Only the refreshed wheel's velocity is recomputed this tick, using the
    // true elapsed time since the last collect for that wheel.  The other
    // wheel's velocity is held from the previous tick (ZOH — not zeroed).
    //
    // refreshedWheel: 0 = none (first iteration or sync fallback — skip all
    //                            velocity updates so both wheels start at 0),
    //                 1 = left wheel was just collected,
    //                 2 = right wheel was just collected.

    if (refreshedWheel == 1) {
        // Left wheel was just collected.
        float encLMm = inputs.encLMm;
        if (_hasTimestampL) {
            float elapsed_s = static_cast<float>(now_ms - _prevTimeMsL) / 1000.0f;
            if (elapsed_s > 0.0f) {
                inputs.velLMms = (encLMm - _prevEncL) / elapsed_s;
            }
        }
        _prevEncL      = encLMm;
        _prevTimeMsL   = now_ms;
        _hasTimestampL = true;
        // Right wheel: ZOH — leave inputs.velRMms unchanged.
    } else if (refreshedWheel == 2) {
        // Right wheel was just collected.
        float encRMm = inputs.encRMm;
        if (_hasTimestampR) {
            float elapsed_s = static_cast<float>(now_ms - _prevTimeMsR) / 1000.0f;
            if (elapsed_s > 0.0f) {
                inputs.velRMms = (encRMm - _prevEncR) / elapsed_s;
            }
        }
        _prevEncR      = encRMm;
        _prevTimeMsR   = now_ms;
        _hasTimestampR = true;
        // Left wheel: ZOH — leave inputs.velLMms unchanged.
    }
    // refreshedWheel == 0: first iteration or no collect — both velocities held at 0.

    // PID runs for BOTH wheels using the held (ZOH) velocities.

    // If no drive command active, ensure motors are stopped.
    if (cmds.tgtLMms == 0.0f && cmds.tgtRMms == 0.0f) {
        cmds.pwmL = 0;
        cmds.pwmR = 0;
        _motorL.setSpeed(0);
        _motorR.setSpeed(0);
        return;
    }

    // PID integrator dt: use the configured control period.
    // The velocity update above already uses the true per-wheel elapsed time for
    // the derivative; the integrator uses the nominal period so integral windup
    // is well-bounded and independent of measurement timing jitter.
    float dt_s = static_cast<float>(_cal.controlPeriodMs) / 1000.0f;
    if (dt_s <= 0.0f) return;

    // Per-wheel velocity PID (Sprint 010 inner loop).
    float uL = _vcL.update(cmds.tgtLMms, inputs.velLMms, dt_s);
    float uR = _vcR.update(cmds.tgtRMms, inputs.velRMms, dt_s);

    cmds.pwmL = static_cast<int16_t>(roundf(uL));
    cmds.pwmR = static_cast<int16_t>(roundf(uR));
    _motorL.setSpeed(static_cast<int8_t>(roundf(uL)));
    _motorR.setSpeed(static_cast<int8_t>(roundf(uR)));
}

void MotorController::getVelocitySourceFlags(bool& leftChip, bool& rightChip) const
{
    // The chip readSpeed (0x47) path was disabled in sprint 013 (motor throb fix).
    // Encoder-delta is the sole velocity source. Always report false.
    leftChip  = false;
    rightChip = false;
}

void MotorController::getEncoderPositions(int32_t& leftMm, int32_t& rightMm) const
{
    // Use atomic reads (request → 4 ms wait → collect) to ensure valid readings
    // outside the split-phase control tick.
    leftMm  = static_cast<int32_t>(_motorL.readEncoderMmFAtomic(_cal));
    rightMm = static_cast<int32_t>(_motorR.readEncoderMmFAtomic(_cal));
}

void MotorController::resetEncoderAccumulators()
{
    _motorL.resetEncoder();
    _motorR.resetEncoder();
    _prevEncL = 0.0f;
    _prevEncR = 0.0f;
    _hasTimestampL = false;
    _hasTimestampR = false;
    _prevTimeMsL   = 0;
    _prevTimeMsR   = 0;
}

float MotorController::clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
