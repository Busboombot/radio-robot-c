#include "MotorController.h"
#include "I2CBus.h"
#include <math.h>
#include <cstdio>

// N13 (030-010): PID_BYPASS macro removed. It was a sprint-014 debug flag
// (open-loop feedforward bypass of the velocity PID) that was always disabled
// (set to 0) and no longer needed now that the encoder-wedge root cause is fixed.

MotorController::MotorController(IMotor& left, IMotor& right, const RobotConfig& cal)
    : _motorL(left), _motorR(right), _cal(cal),
      _vcL(cal.velKff, cal.velKp, cal.velKi, cal.velIMax, cal.minWheelMms, cal.velKaw),
      _vcR(cal.velKff, cal.velKp, cal.velKi, cal.velIMax, cal.minWheelMms, cal.velKaw),
      _cmdEncStartL(0.0f), _cmdEncStartR(0.0f),
      _cmdRatio(1.0f), _fasterIsRight(false),
      _cmds(nullptr),
      _prevEncL(0.0f), _prevEncR(0.0f),
      _prevTimeMsL(0), _prevTimeMsR(0),
      _hasTimestampL(false), _hasTimestampR(false),
      _lastPidMs(0), _hasPidTick(false),
      _wedgePrevEncL(0.0f), _wedgePrevEncR(0.0f),
      _wedgePrevValidL(false), _wedgePrevValidR(false),
      _stuckCountL(0), _stuckCountR(0),
      _wedgeEmittedL(false), _wedgeEmittedR(false),
      _hasMovedL(false), _hasMovedR(false),
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
    _stuckCountL     = 0;
    _stuckCountR     = 0;
    _wedgeEmittedL   = false;
    _wedgeEmittedR   = false;
    _wedgePrevValidL = false;
    _wedgePrevValidR = false;
    _hasMovedL       = false;  // (033-005d) re-arm grace latches
    _hasMovedR       = false;
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
    _vcL.reset();
    _vcR.reset();
    // (033-005d) Clear arming-grace latches: the detector must not fire until
    // each wheel has moved at least once since this command started.
    _hasMovedL = false;
    _hasMovedR = false;
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

    _fasterIsRight = newFasterIsRight;
    _cmdRatio = newRatio;
    // (033-005d) Clear arming-grace latches on streaming command start.
    _hasMovedL = false;
    _hasMovedR = false;
}

void MotorController::stop()
{
    if (_cmds) {
        _cmds->tgtLMms = 0.0f;
        _cmds->tgtRMms = 0.0f;
    }
    _vcL.reset();
    _vcR.reset();
    // Use the control loop's cached encoder values (not a fresh atomic read,
    // which — butted against the 0x5F stop below — wedges the Nezha encoder).
    _cmdEncStartL = _prevEncL;
    _cmdEncStartR = _prevEncR;
    _motorL.setSpeed(0);
    _motorR.setSpeed(0);
    // (033-005d) Clear arming-grace latches on stop.
    _hasMovedL = false;
    _hasMovedR = false;
}

void MotorController::resetIntegrators()
{
    _vcL.reset();
    _vcR.reset();
}

void MotorController::updateVelGains(const RobotConfig& cal)
{
    // Push the (possibly SET-modified) per-wheel velocity gains into both live
    // VelocityControllers. Without this, SET vel.kP/kI/kFF/iMax/kAw only changed
    // the config struct, not the running controllers (which hold copies made at
    // construction). velFiltAlpha and syncGain are read from _cal each tick so
    // they do not need pushing here.
    _vcL.kFF = cal.velKff;  _vcR.kFF = cal.velKff;
    _vcL.kP  = cal.velKp;   _vcR.kP  = cal.velKp;
    _vcL.kI  = cal.velKi;   _vcR.kI  = cal.velKi;
    _vcL.iMax = cal.velIMax; _vcR.iMax = cal.velIMax;
    _vcL.kAw  = cal.velKaw;  _vcR.kAw  = cal.velKaw;
    _vcL.minWheelMms = cal.minWheelMms; _vcR.minWheelMms = cal.minWheelMms;
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
    } else if (refreshedWheel == 3) {
        // Both wheels updated this tick (WedgeTest pattern — sprint 015).
        float encLMm = inputs.encLMm;
        if (!_hasTimestampL) {
            _prevEncL = encLMm; _prevTimeMsL = now_ms; _hasTimestampL = true;
        } else {
            float elapsed_s = static_cast<float>(now_ms - _prevTimeMsL) / 1000.0f;
            if (elapsed_s > 0.0f) {
                float rawV = (encLMm - _prevEncL) / elapsed_s;
                if (fabsf(rawV) <= kMaxPlausibleMmps) {
                    float a = _cal.velFiltAlpha;
                    inputs.velLMms = a * rawV + (1.0f - a) * inputs.velLMms;
                    _prevEncL    = encLMm;
                    _prevTimeMsL = now_ms;
                }
            }
        }
        float encRMm = inputs.encRMm;
        if (!_hasTimestampR) {
            _prevEncR = encRMm; _prevTimeMsR = now_ms; _hasTimestampR = true;
        } else {
            float elapsed_s = static_cast<float>(now_ms - _prevTimeMsR) / 1000.0f;
            if (elapsed_s > 0.0f) {
                float rawV = (encRMm - _prevEncR) / elapsed_s;
                if (fabsf(rawV) <= kMaxPlausibleMmps) {
                    float a = _cal.velFiltAlpha;
                    inputs.velRMms = a * rawV + (1.0f - a) * inputs.velRMms;
                    _prevEncR    = encRMm;
                    _prevTimeMsR = now_ms;
                }
            }
        }
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
        // Left-wheel check — when left (or both) was just collected.
        if (refreshedWheel == 1 || refreshedWheel == 3) {
            float encL = inputs.encLMm;
            if (cmds.tgtLMms != 0.0f) {
                if (_wedgePrevValidL && encL != _wedgePrevEncL) {
                    // Encoder moved: re-arm and set the arming-grace latch.
                    _stuckCountL   = 0;
                    _wedgeEmittedL = false;
                    _hasMovedL     = true;  // (033-005d) wheel has moved at least once
                } else if (_wedgePrevValidL && encL == _wedgePrevEncL) {
                    // (033-005d) Only count toward wedge threshold once the
                    // wheel has moved at least once since the command started.
                    // This prevents spin-up lag from firing the detector.
                    if (_hasMovedL) {
                        if (_stuckCountL < 255) ++_stuckCountL;
                    }
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
#ifndef HOST_BUILD
                    uint32_t busErr    = _i2cBus ? (_i2cBus->errCount(0x10)) : 0;
                    uint32_t reentryN  = _i2cBus ? (_i2cBus->reentryViolations()) : 0;
                    int      lastErrV  = _i2cBus ? (_i2cBus->lastErr(0x10)) : 0;
#else
                    uint32_t busErr = 0, reentryN = 0;
                    int lastErrV = 0;
#endif
                    // (033-005c) Include a fresh raw read alongside the filtered
                    // value.  raw frozen + enc frozen → real chip/I2C wedge or
                    // stall; raw moving + enc frozen → outlier-filter hold.
                    // readEncoderMmFSettle is used (not Atomic) to avoid the
                    // extra 4 ms pre-write idle during the EVT emit path.
                    int rawL = (int)_motorL.readEncoderMmFSettle(_cal);
                    char evtBuf[112];
                    snprintf(evtBuf, sizeof(evtBuf),
                             "EVT enc_wedged wheel=L enc=%d raw=%d n=%u err=%lu reentry=%lu lastErr=%d",
                             (int)encL,
                             rawL,
                             (unsigned)_stuckCountL,
                             (unsigned long)busErr,
                             (unsigned long)reentryN,
                             lastErrV);
                    (*_evtFn)(evtBuf, *_evtCtx);
                }
            }
        }

        // Right-wheel check — when right (or both) was just collected.
        if (refreshedWheel == 2 || refreshedWheel == 3) {
            float encR = inputs.encRMm;
            if (cmds.tgtRMms != 0.0f) {
                if (_wedgePrevValidR && encR != _wedgePrevEncR) {
                    // Encoder moved: re-arm and set the arming-grace latch.
                    _stuckCountR   = 0;
                    _wedgeEmittedR = false;
                    _hasMovedR     = true;  // (033-005d) wheel has moved at least once
                } else if (_wedgePrevValidR && encR == _wedgePrevEncR) {
                    // (033-005d) Gate on arming grace: don't count until moved once.
                    if (_hasMovedR) {
                        if (_stuckCountR < 255) ++_stuckCountR;
                    }
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
#ifndef HOST_BUILD
                    uint32_t busErr    = _i2cBus ? (_i2cBus->errCount(0x10)) : 0;
                    uint32_t reentryN  = _i2cBus ? (_i2cBus->reentryViolations()) : 0;
                    int      lastErrV  = _i2cBus ? (_i2cBus->lastErr(0x10)) : 0;
#else
                    uint32_t busErr = 0, reentryN = 0;
                    int lastErrV = 0;
#endif
                    // (033-005c) Include a fresh raw read alongside the filtered
                    // value.  raw frozen + enc frozen → real chip/I2C wedge or
                    // stall; raw moving + enc frozen → outlier-filter hold.
                    int rawR = (int)_motorR.readEncoderMmFSettle(_cal);
                    char evtBuf[112];
                    snprintf(evtBuf, sizeof(evtBuf),
                             "EVT enc_wedged wheel=R enc=%d raw=%d n=%u err=%lu reentry=%lu lastErr=%d",
                             (int)encR,
                             rawR,
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
        // Clear stale EMA velocity: MockMotor stops instantly, so the
        // measurement should reflect 0 immediately rather than freezing
        // at the last filtered value until the next drive command.
        inputs.velLMms = 0.0f;
        inputs.velRMms = 0.0f;
        return;
    }

    // PID integrator dt: the ACTUAL elapsed control-tick time, not the nominal
    // controlPeriodMs. The real loop runs at ~24 ms (10 ms nominal + 2x4 ms
    // encoder settle + bus time), so using the 10 ms nominal made kI accumulate
    // at ~0.4x strength and never close the steady-state error (wheels held ~190
    // of a 200 mm/s command). Clamp the measured delta to [5, 50] ms so a stalled
    // or first tick can't spike the integrator (preserves windup bounding).
    float dt_s;
    if (_hasPidTick) {
        int32_t dms = static_cast<int32_t>(now_ms - _lastPidMs);
        if (dms < 5)  dms = 5;
        if (dms > 50) dms = 50;
        dt_s = static_cast<float>(dms) / 1000.0f;
    } else {
        dt_s = static_cast<float>(_cal.controlPeriodMs) / 1000.0f;
        _hasPidTick = true;
    }
    _lastPidMs = now_ms;
    if (dt_s <= 0.0f) return;

    // Cross-wheel coupling — "slowest wheel governs" (015). Computed BEFORE the
    // per-wheel PID by adjusting the effective setpoints (not the PWM), so the
    // per-wheel PID does the work and there is no fighting. The wheel that is
    // ACHIEVING more of its target is slaved toward the slower wheel's pace;
    // disturbing one wheel pulls the other onto the ratio line. A deadband lets
    // the per-wheel PID absorb LIGHT touches; only a real, sustained discrepancy
    // couples. SET sync=0 -> independent.
    //
    // Blend fraction = syncGain * (discrepancy - deadband), clamped [0,1].
    // At syncGain=1 the coupled target reaches the fully-matched value only when
    // the discrepancy is 100 % (one wheel fully stopped).  Proportional blending
    // prevents the bang-bang setpoint switch that caused oscillation.
    float effTgtL = cmds.tgtLMms;
    float effTgtR = cmds.tgtRMms;
    // Only couple when BOTH wheels drive the SAME direction (straight / curve).
    // For a spin-in-place the targets are opposite-sign (tgtL=-X, tgtR=+X); the
    // "slowest-wheel-governs" math (coupled = velOther/ratio, ratio<0) then
    // collapses the faster wheel toward the lagging one and the spin degenerates
    // to a single wheel. Same-sign-only (product > 0) keeps spins independent.
    if (_cal.syncGain > 0.0f && cmds.tgtLMms * cmds.tgtRMms > 0.0f) {
        float ratio = cmds.tgtRMms / cmds.tgtLMms;        // commanded vR/vL
        float achL  = inputs.velLMms / cmds.tgtLMms;      // fraction-of-target each wheel does
        float achR  = inputs.velRMms / cmds.tgtRMms;
        const float deadband = 0.08f;
        float disc = achL - achR;                         // positive = left ahead
        if (disc > deadband) {
            float blend = _cal.syncGain * (disc - deadband);
            if (blend > 1.0f) blend = 1.0f;
            float coupled = inputs.velRMms / ratio;       // fully-matched target
            effTgtL = cmds.tgtLMms * (1.0f - blend) + coupled * blend;
        } else if (-disc > deadband) {
            float blend = _cal.syncGain * (-disc - deadband);
            if (blend > 1.0f) blend = 1.0f;
            float coupled = inputs.velLMms * ratio;
            effTgtR = cmds.tgtRMms * (1.0f - blend) + coupled * blend;
        }
    }

    // Per-wheel velocity PID (Sprint 010 inner loop) on the (possibly coupled) targets.
    float uL = _vcL.update(effTgtL, inputs.velLMms, dt_s);
    float uR = _vcR.update(effTgtR, inputs.velRMms, dt_s);

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
