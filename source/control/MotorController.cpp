#include "MotorController.h"
#include "I2CBus.h"
#include <math.h>
#include <cstdio>

// DEBUG (sprint 014 — encoder-wedge isolation): when 1, controlTick() bypasses
// the velocity PID and drives the wheels OPEN-LOOP at a fixed PWM proportional
// to the commanded velocity (feedforward only). The motor command does NOT
// react to measured velocity, so an encoder reading 0 cannot make the PID slam
// the PWM. Isolates whether PID feedback perpetuates the wedge.
// Set back to 0 to restore closed-loop PID.
#define PID_BYPASS 0

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
      _hasTimestampL(false), _hasTimestampR(false),
      _wedgePrevEncL(0.0f), _wedgePrevEncR(0.0f),
      _wedgePrevValidL(false), _wedgePrevValidR(false),
      _stuckCountL(0), _stuckCountR(0),
      _wedgeEmittedL(false), _wedgeEmittedR(false),
      _i2cBus(nullptr),
      _evtFn(nullptr), _evtCtx(nullptr)
{
    gains.kFF     = 0.15f;
    gains.kP      = 0.05f;
    gains.kI      = 0.20f;
    gains.iClamp  = 60.0f;
    gains.kRatio  = 0.01f;
}

void MotorController::resetStuckCounters()
{
    _stuckCountL    = 0;
    _stuckCountR    = 0;
    _wedgeEmittedL  = false;
    _wedgeEmittedR  = false;
    _wedgePrevValidL = false;
    _wedgePrevValidR = false;
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
    // Use the control loop's cached encoder values (not a fresh atomic read,
    // which wedges the Nezha encoder — see encoder-wedge note).
    _cmdEncStartL = _prevEncL;
    _cmdEncStartR = _prevEncR;
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

    // Use the control loop's cached encoder values (updated every tick) — NOT a
    // fresh atomic read. Firing an atomic 0x46 read from this comms-path call,
    // butted against the control task's own reads / the 0x5F stop, wedges the
    // Nezha encoder. See docs/knowledge encoder-wedge note.
    float curL = _prevEncL;
    float curR = _prevEncR;
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
    // Use the control loop's cached encoder values (not a fresh atomic read,
    // which — butted against the 0x5F stop below — wedges the Nezha encoder).
    _cmdEncStartL = _prevEncL;
    _cmdEncStartR = _prevEncR;
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

    // Max physically-plausible wheel speed. The robot tops out near ~400 mm/s;
    // an occasional corrupt encoder read produces a huge bogus delta (seen: tens
    // of thousands of mm/s). Reject any sample beyond this bound — keep the prev
    // position/time so the NEXT good read computes a correct delta — then EMA the
    // accepted samples.
    static constexpr float kMaxPlausibleMmps = 1000.0f;

    if (refreshedWheel == 1) {
        // Left wheel was just collected.
        float encLMm = inputs.encLMm;
        if (!_hasTimestampL) {
            _prevEncL = encLMm; _prevTimeMsL = now_ms; _hasTimestampL = true;
        } else {
            float elapsed_s = static_cast<float>(now_ms - _prevTimeMsL) / 1000.0f;
            if (elapsed_s > 0.0f) {
                float rawV = (encLMm - _prevEncL) / elapsed_s;
                if (fabsf(rawV) <= kMaxPlausibleMmps) {        // accept plausible
                    float a = _cal.velFiltAlpha;               // EMA smoothing
                    inputs.velLMms = a * rawV + (1.0f - a) * inputs.velLMms;
                    _prevEncL    = encLMm;
                    _prevTimeMsL = now_ms;
                }
                // else: garbage read — reject, hold velLMms and prev refs.
            }
        }
        // Right wheel: ZOH — leave inputs.velRMms unchanged.
    } else if (refreshedWheel == 2) {
        // Right wheel was just collected.
        float encRMm = inputs.encRMm;
        if (!_hasTimestampR) {
            _prevEncR = encRMm; _prevTimeMsR = now_ms; _hasTimestampR = true;
        } else {
            float elapsed_s = static_cast<float>(now_ms - _prevTimeMsR) / 1000.0f;
            if (elapsed_s > 0.0f) {
                float rawV = (encRMm - _prevEncR) / elapsed_s;
                if (fabsf(rawV) <= kMaxPlausibleMmps) {        // accept plausible
                    float a = _cal.velFiltAlpha;               // EMA smoothing
                    inputs.velRMms = a * rawV + (1.0f - a) * inputs.velRMms;
                    _prevEncR    = encRMm;
                    _prevTimeMsR = now_ms;
                }
                // else: garbage read — reject, hold velRMms and prev refs.
            }
        }
        // Left wheel: ZOH — leave inputs.velLMms unchanged.
    }
    // refreshedWheel == 0: first iteration or no collect — both velocities held at 0.

    // PID runs for BOTH wheels using the held (ZOH) velocities.

    // -------------------------------------------------------------------------
    // Encoder-wedge detector (015-003).
    //
    // Per-wheel: if the commanded speed is non-zero and the encoder value has
    // not changed since the last reading, increment the stuck counter. When
    // the counter reaches kWedgeThreshold and the latch is clear, emit
    // EVT enc_wedged once (latched) and set the latch. Re-arm when the
    // encoder moves.
    //
    // Only checked when a wheel's encoder was just refreshed (refreshedWheel
    // matches the wheel index) so we compare two real hardware reads, not
    // stale ZOH-held values.
    //
    // See: .clasi/issues/residual-motor-encoder-wedge-after-stop.md
    // -------------------------------------------------------------------------
    {
        // Left-wheel check — only when left was just collected.
        if (refreshedWheel == 1) {
            float encL = inputs.encLMm;
            if (cmds.tgtLMms != 0.0f) {
                if (_wedgePrevValidL && encL == _wedgePrevEncL) {
                    if (_stuckCountL < 255) ++_stuckCountL;
                } else {
                    // Encoder moved — re-arm.
                    _stuckCountL   = 0;
                    _wedgeEmittedL = false;
                }
            } else {
                // Not commanded — reset.
                _stuckCountL   = 0;
                _wedgeEmittedL = false;
            }
            _wedgePrevEncL   = encL;
            _wedgePrevValidL = true;

            if (_stuckCountL >= kWedgeThreshold && !_wedgeEmittedL) {
                _wedgeEmittedL = true;
                if (_evtFn && *_evtFn && _evtCtx && *_evtCtx) {
                    uint32_t busErr    = _i2cBus ? (_i2cBus->errCount(0x10)) : 0;
                    uint32_t reentryN  = _i2cBus ? (_i2cBus->reentryViolations()) : 0;
                    int      lastErrV  = _i2cBus ? (_i2cBus->lastErr(0x10)) : 0;
                    char evtBuf[96];
                    snprintf(evtBuf, sizeof(evtBuf),
                             "EVT enc_wedged wheel=L enc=%d n=%u err=%lu reentry=%lu lastErr=%d",
                             (int)encL,
                             (unsigned)_stuckCountL,
                             (unsigned long)busErr,
                             (unsigned long)reentryN,
                             lastErrV);
                    (*_evtFn)(evtBuf, *_evtCtx);
                }
            }
        }

        // Right-wheel check — only when right was just collected.
        if (refreshedWheel == 2) {
            float encR = inputs.encRMm;
            if (cmds.tgtRMms != 0.0f) {
                if (_wedgePrevValidR && encR == _wedgePrevEncR) {
                    if (_stuckCountR < 255) ++_stuckCountR;
                } else {
                    _stuckCountR   = 0;
                    _wedgeEmittedR = false;
                }
            } else {
                _stuckCountR   = 0;
                _wedgeEmittedR = false;
            }
            _wedgePrevEncR   = encR;
            _wedgePrevValidR = true;

            if (_stuckCountR >= kWedgeThreshold && !_wedgeEmittedR) {
                _wedgeEmittedR = true;
                if (_evtFn && *_evtFn && _evtCtx && *_evtCtx) {
                    uint32_t busErr    = _i2cBus ? (_i2cBus->errCount(0x10)) : 0;
                    uint32_t reentryN  = _i2cBus ? (_i2cBus->reentryViolations()) : 0;
                    int      lastErrV  = _i2cBus ? (_i2cBus->lastErr(0x10)) : 0;
                    char evtBuf[96];
                    snprintf(evtBuf, sizeof(evtBuf),
                             "EVT enc_wedged wheel=R enc=%d n=%u err=%lu reentry=%lu lastErr=%d",
                             (int)encR,
                             (unsigned)_stuckCountR,
                             (unsigned long)busErr,
                             (unsigned long)reentryN,
                             lastErrV);
                    (*_evtFn)(evtBuf, *_evtCtx);
                }
            }
        }
    }

    // If no drive command active, ensure motors are stopped.
    if (cmds.tgtLMms == 0.0f && cmds.tgtRMms == 0.0f) {
        cmds.pwmL = 0;
        cmds.pwmR = 0;
        _motorL.setSpeed(0);
        _motorR.setSpeed(0);
        return;
    }

#if PID_BYPASS
    // DEBUG: open-loop feedforward — PWM% = target_mm_s * velKff, clamped ±100.
    // setSpeed() itself is write-on-change (Motor level), so just call it every
    // tick unconditionally — no local change-tracking here (a stale static was
    // skipping the restart write after a stop).
    {
        float ffL = cmds.tgtLMms * _cal.velKff;
        float ffR = cmds.tgtRMms * _cal.velKff;
        if (ffL >  100.0f) ffL =  100.0f;
        if (ffL < -100.0f) ffL = -100.0f;
        if (ffR >  100.0f) ffR =  100.0f;
        if (ffR < -100.0f) ffR = -100.0f;
        int8_t pL = (int8_t)roundf(ffL);
        int8_t pR = (int8_t)roundf(ffR);
        cmds.pwmL = pL;
        cmds.pwmR = pR;
        _motorL.setSpeed(pL);
        _motorR.setSpeed(pR);
        return;
    }
#endif

    // PID integrator dt: use the configured control period.
    // The velocity update above already uses the true per-wheel elapsed time for
    // the derivative; the integrator uses the nominal period so integral windup
    // is well-bounded and independent of measurement timing jitter.
    float dt_s = static_cast<float>(_cal.controlPeriodMs) / 1000.0f;
    if (dt_s <= 0.0f) return;

    // Per-wheel velocity PID (Sprint 010 inner loop).
    float uL = _vcL.update(cmds.tgtLMms, inputs.velLMms, dt_s);
    float uR = _vcR.update(cmds.tgtRMms, inputs.velRMms, dt_s);

    // Cross-wheel ratio coupling (015): pull the wheels onto the commanded ratio
    // line so disturbing one wheel slows the other (keeps the phase-plot point on
    // the diagonal). syncErr > 0 means the right wheel is slower than the ratio
    // demands; we then push right harder and slow left, and vice-versa. A light
    // touch yields a small syncErr (absorbed by the per-wheel PID); a hard/held
    // wheel yields a large syncErr that pulls both wheels down together.
    // syncGain = 0 restores fully-independent per-wheel control.
    float ratio   = (cmds.tgtLMms != 0.0f) ? (cmds.tgtRMms / cmds.tgtLMms) : 1.0f;
    float syncErr = (inputs.velLMms * ratio) - inputs.velRMms;
    float corr    = _cal.syncGain * syncErr;
    uR += corr;
    uL -= corr;

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
