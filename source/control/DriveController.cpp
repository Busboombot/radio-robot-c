// DriveController.cpp — S/T/D/G drive state machines, S-mode watchdog,
// streaming encoder counter, and odometry delta tracking.
//
// Transplanted from CommandProcessor.cpp (Sprint 007, Ticket 003).
// All speeds in mm/s; distances in mm.

#include "DriveController.h"
#include "MotorController.h"
#include "Odometry.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

DriveController::DriveController(MotorController& mc, Odometry& odo, const RobotConfig& cfg)
    : _mc(mc)
    , _odo(odo)
    , _cfg(cfg)
    , _mode(DriveMode::IDLE)
    , _driveFn(nullptr)
    , _driveCtx(nullptr)
    , _corrId{}
    , _lastSMs(0)
    , _tgtL(0.0f)
    , _tgtR(0.0f)
    , _tEndMs(0)
    , _dEncStartL(0)
    , _dEncStartR(0)
    , _dTargetMm(0)
    , _dTimeoutMs(0)
    , _gPhase(GPhase::IDLE)
    , _gTargetX(0.0f)
    , _gTargetY(0.0f)
    , _gSpeed(0.0f)
    , _gArcLeftMm(0.0f)
    , _gArcRightMm(0.0f)
    , _gArcStartL(0.0f)
    , _gArcStartR(0.0f)
    , _lastTickMs(0)
    , _currentTimeMs(0)
    , _prevOdoEncL(0)
    , _prevOdoEncR(0)
{
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

void DriveController::beginStream(float leftMms, float rightMms, uint32_t now_ms,
                                   ReplyFn fn, void* ctx)
{
    _mc.startDrive(leftMms, rightMms);
    _mc.setTarget(leftMms, rightMms);
    _tgtL    = leftMms;
    _tgtR    = rightMms;
    _mode    = DriveMode::STREAMING;
    _lastSMs = now_ms;
    _driveFn  = fn;
    _driveCtx = ctx;
}

void DriveController::beginTimed(float leftMms, float rightMms,
                                  uint32_t durationMs, uint32_t now_ms,
                                  ReplyFn fn, void* ctx, const char* corr_id)
{
    _mc.startDriveClean(leftMms, rightMms);
    _mc.setTarget(leftMms, rightMms);
    _tgtL     = leftMms;
    _tgtR     = rightMms;
    _tEndMs   = _lastTickMs + durationMs;
    _mode     = DriveMode::TIMED;
    _driveFn  = fn;
    _driveCtx = ctx;
    if (corr_id && corr_id[0] != '\0') {
        strncpy(_corrId, corr_id, sizeof(_corrId) - 1);
        _corrId[sizeof(_corrId) - 1] = '\0';
    } else {
        _corrId[0] = '\0';
    }
    (void)now_ms;
}

void DriveController::beginDistance(float leftMms, float rightMms,
                                     int32_t targetMm, uint32_t now_ms,
                                     ReplyFn fn, void* ctx, const char* corr_id)
{
    _mc.startDriveClean(leftMms, rightMms);
    _mc.setTarget(leftMms, rightMms);
    _tgtL = leftMms;
    _tgtR = rightMms;
    _mc.resetEncoderAccumulators();
    _mc.getEncoderPositions(_dEncStartL, _dEncStartR);
    _dTargetMm  = targetMm;
    _dTimeoutMs = _lastTickMs + 5000;
    _mode       = DriveMode::DISTANCE;
    _driveFn    = fn;
    _driveCtx   = ctx;
    if (corr_id && corr_id[0] != '\0') {
        strncpy(_corrId, corr_id, sizeof(_corrId) - 1);
        _corrId[sizeof(_corrId) - 1] = '\0';
    } else {
        _corrId[0] = '\0';
    }
    (void)now_ms;
}

void DriveController::beginGoTo(float tx, float ty, float speedMms, uint32_t now_ms,
                                 ReplyFn fn, void* ctx, const char* corr_id)
{
    _gTargetX = tx;
    _gTargetY = ty;
    _gSpeed   = speedMms;

    float angleRad = atan2f(ty, tx);
    float kgt      = _cfg.turnThresholdMm;
    float angleDeg = angleRad * 57.2957795f;

    if (fabsf(angleDeg) > kgt) {
        // Pre-rotate phase: rotate in place to face target
        float turnSign = (ty >= 0.0f) ? 1.0f : -1.0f;
        _mc.startDriveClean(-turnSign * speedMms, turnSign * speedMms);
        _mc.setTarget(-turnSign * speedMms, turnSign * speedMms);
        _tgtL = -turnSign * speedMms;
        _tgtR =  turnSign * speedMms;
        float tw     = _cfg.trackwidthMm;
        _gArcLeftMm  = -turnSign * (tw / 2.0f) * fabsf(angleRad);
        _gArcRightMm =  turnSign * (tw / 2.0f) * fabsf(angleRad);
        int32_t el, er;
        _mc.getEncoderPositions(el, er);
        _gArcStartL = (float)el;
        _gArcStartR = (float)er;
        _gPhase = GPhase::PRE_ROTATE;
    } else {
        // Arc phase directly (shallow angle)
        float tw = _cfg.trackwidthMm;
        computeArc(tx, ty, tw, _gArcLeftMm, _gArcRightMm);
        float maxArc   = fmaxf(fabsf(_gArcLeftMm), fabsf(_gArcRightMm));
        float leftSpd  = (maxArc > 0.001f) ? (speedMms * _gArcLeftMm  / maxArc) : speedMms;
        float rightSpd = (maxArc > 0.001f) ? (speedMms * _gArcRightMm / maxArc) : speedMms;
        _mc.startDriveClean(leftSpd, rightSpd);
        _mc.setTarget(leftSpd, rightSpd);
        _tgtL = leftSpd;
        _tgtR = rightSpd;
        int32_t el, er;
        _mc.getEncoderPositions(el, er);
        _gArcStartL = (float)el;
        _gArcStartR = (float)er;
        _gPhase = GPhase::ARC;
    }

    _mode     = DriveMode::GO_TO;
    _driveFn  = fn;
    _driveCtx = ctx;
    if (corr_id && corr_id[0] != '\0') {
        strncpy(_corrId, corr_id, sizeof(_corrId) - 1);
        _corrId[sizeof(_corrId) - 1] = '\0';
    } else {
        _corrId[0] = '\0';
    }
    (void)now_ms;
}

void DriveController::stop(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    fullStop(fn, ctx);
    (void)now_ms;
}

// ---------------------------------------------------------------------------
// tick
// ---------------------------------------------------------------------------

void DriveController::tick(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    // Throttle to tickMs cadence
    int32_t tickMs = _cfg.tickMs;
    if ((now_ms - _lastTickMs) < (uint32_t)tickMs) return;

    float dt_s      = (float)(now_ms - _lastTickMs) / 1000.0f;
    _lastTickMs     = now_ms;
    _currentTimeMs  = now_ms;

    // Run motor controller and update odometry
    if (_mode != DriveMode::IDLE) {
        _mc.tick(dt_s);

        int32_t encL, encR;
        _mc.getEncoderPositions(encL, encR);
        float dL = (float)(encL - _prevOdoEncL);
        float dR = (float)(encR - _prevOdoEncR);
        _prevOdoEncL = encL;
        _prevOdoEncR = encR;
        _odo.update(dL, dR, _cfg.trackwidthMm);
    }

    // Convenience: drive sink (for async completions) vs active sink (for streaming).
    // _driveFn/_driveCtx: captured when the drive began — routes completions to
    // the channel that originated the command.
    // fn/ctx: the active channel sink — used for streaming telemetry only.
    ReplyFn  dfn = _driveFn  ? _driveFn  : fn;
    void*    dct = _driveFn  ? _driveCtx : ctx;

    // Helper: build an EVT line, appending " #<id>" when _corrId is set.
    // Uses a local buffer on the stack; safe because dfn() is called inline.
    auto emitEvt = [&](const char* base) {
        if (_corrId[0] != '\0') {
            char evtBuf[64];
            snprintf(evtBuf, sizeof(evtBuf), "%s #%s", base, _corrId);
            dfn(evtBuf, dct);
        } else {
            dfn(base, dct);
        }
        _corrId[0] = '\0';  // clear after emitting
    };

    // S-mode watchdog
    if (_mode == DriveMode::STREAMING) {
        if ((now_ms - _lastSMs) > (uint32_t)_cfg.sTimeoutMs) {
            fullStop(dfn, dct);
            emitEvt("EVT safety_stop");
        }
    }

    // T-mode: stop when deadline reached
    if (_mode == DriveMode::TIMED && now_ms >= _tEndMs) {
        fullStop(dfn, dct);
        emitEvt("EVT done T");
    }

    // D-mode: stop when average encoder travel >= target, or on timeout
    if (_mode == DriveMode::DISTANCE) {
        int32_t l, r;
        _mc.getEncoderPositions(l, r);
        int32_t traveled = (abs(l - _dEncStartL) + abs(r - _dEncStartR)) / 2;
        if (traveled >= _dTargetMm || now_ms >= _dTimeoutMs) {
            fullStop(dfn, dct);
            emitEvt("EVT done D");
        }
    }

    // G-mode: advance go-to state machine
    if (_mode == DriveMode::GO_TO) {
        int32_t el, er;
        _mc.getEncoderPositions(el, er);
        float kgd = _cfg.doneTolMm;

        if (_gPhase == GPhase::PRE_ROTATE) {
            float dL      = fabsf((float)el - _gArcStartL);
            float dR      = fabsf((float)er - _gArcStartR);
            float targetL = fabsf(_gArcLeftMm);
            float targetR = fabsf(_gArcRightMm);
            bool doneL = dL >= targetL - kgd;
            bool doneR = dR >= targetR - kgd;
            if (doneL && doneR) {
                float tw = _cfg.trackwidthMm;
                computeArc(_gTargetX, _gTargetY, tw, _gArcLeftMm, _gArcRightMm);
                float maxArc   = fmaxf(fabsf(_gArcLeftMm), fabsf(_gArcRightMm));
                float leftSpd  = (maxArc > 0.001f) ? (_gSpeed * _gArcLeftMm  / maxArc) : _gSpeed;
                float rightSpd = (maxArc > 0.001f) ? (_gSpeed * _gArcRightMm / maxArc) : _gSpeed;
                _mc.startDriveClean(leftSpd, rightSpd);
                _mc.setTarget(leftSpd, rightSpd);
                _tgtL = leftSpd;
                _tgtR = rightSpd;
                _gArcStartL = (float)el;
                _gArcStartR = (float)er;
                _gPhase = GPhase::ARC;
            }
        } else if (_gPhase == GPhase::ARC) {
            float dL = (float)el - _gArcStartL;
            float dR = (float)er - _gArcStartR;
            bool doneL = fabsf(dL - _gArcLeftMm)  <= kgd;
            bool doneR = fabsf(dR - _gArcRightMm) <= kgd;
            if (doneL && doneR) {
                fullStop(dfn, dct);
                _gPhase = GPhase::IDLE;
                emitEvt("EVT done G");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void DriveController::fullStop(ReplyFn fn, void* ctx)
{
    _mc.stop();
    _mode  = DriveMode::IDLE;
    _tgtL  = 0.0f;
    _tgtR  = 0.0f;
    (void)fn;
    (void)ctx;
}

/**
 * Compute differential arc wheel distances for a relative XY target.
 * Robot starts at (0,0,0). Heading=0 is forward (+X direction).
 *
 * @param tx           Target X in mm (forward from robot)
 * @param ty           Target Y in mm (left from robot)
 * @param trackwidthMm Distance between wheel contact patches in mm
 * @param leftMm       Output: left wheel distance in mm (signed)
 * @param rightMm      Output: right wheel distance in mm (signed)
 */
void DriveController::computeArc(float tx, float ty, float trackwidthMm,
                                  float& leftMm, float& rightMm)
{
    float W = trackwidthMm;
    if (fabsf(ty) < 0.001f) {
        leftMm  = tx;
        rightMm = tx;
        return;
    }
    float R     = (tx * tx + ty * ty) / (2.0f * ty);
    float alpha = atan2f(ty, tx + R);
    leftMm  = (R - W / 2.0f) * alpha;
    rightMm = (R + W / 2.0f) * alpha;
}
