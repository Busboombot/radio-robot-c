// BodyVelocityController.cpp — body-level (v, ω) motion profiler.
//
// See BodyVelocityController.h for full API documentation.
// Architecture reference: .clasi/sprints/017-.../architecture-update.md
// Sprint 017, Ticket 002.

#include "BodyVelocityController.h"
#include "MotorController.h"
#include "BodyKinematics.h"
#include <math.h>

// ---------------------------------------------------------------------------
// M_PI guard — micro:bit / ARMCC may not define M_PI by default.
// ---------------------------------------------------------------------------
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float kDegToRad = (float)(M_PI / 180.0);

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

BodyVelocityController::BodyVelocityController(MotorController& mc,
                                                 const RobotConfig& cfg)
    : _mc(mc), _cfg(cfg),
      _v(0.0f), _omega(0.0f), _vTgt(0.0f), _omegaTgt(0.0f),
      _aLive(0.0f), _omegaALive(0.0f)
{
}

// ---------------------------------------------------------------------------
// Setters
// ---------------------------------------------------------------------------

void BodyVelocityController::setTarget(float v_mms, float omega_rads)
{
    _vTgt      = v_mms;
    _omegaTgt  = omega_rads;
}

// ---------------------------------------------------------------------------
// Profile step
// ---------------------------------------------------------------------------

bool BodyVelocityController::advance(float dt_s)
{
    if (dt_s <= 0.0f) {
        return !atTarget();
    }

    // ------------------------------------------------------------------
    // Linear channel — asymmetric accel/decel with optional jerk limit.
    //
    // At jMax == 0 (default): pure trapezoid — approach v directly under
    // the per-tick dv_max step (identical to pre-018 behaviour).
    //
    // At jMax > 0: S-curve — slew _aLive toward the demanded acceleration
    // under the jerk bound (jMax * dt_s), then integrate v += _aLive*dt_s.
    //
    // "Demanded accel" = +aMax when target is farther from zero than v
    // (speeding up), -aDecel otherwise (slowing down).
    // ------------------------------------------------------------------
    float vTgtClamped = clamp(_vTgt, -_cfg.vBodyMax, +_cfg.vBodyMax);

    if (_cfg.jMax > 0.0f) {
        // S-curve path: jerk-limit the acceleration, then integrate toward
        // the target using approach() so the integration cannot overshoot.
        //
        // aTarget: +aMax when v must increase to reach vTgtClamped, else -aDecel.
        float aTarget = (vTgtClamped >= _v) ? _cfg.aMax : -_cfg.aDecel;
        float jerkStep = _cfg.jMax * dt_s;
        _aLive = approach(_aLive, aTarget, jerkStep);
        // Integrate, but cap the step so we never overshoot vTgtClamped.
        // fabsf guards against negative _aLive pointing away from target.
        _v = approach(_v, vTgtClamped, fabsf(_aLive * dt_s));
    } else {
        // Trapezoid path (jMax == 0): identical to pre-018 behaviour.
        // Use aDecel when the clamped target is closer to zero than current v
        // (i.e. we are decelerating), otherwise aMax.
        float dv_max = (fabsf(vTgtClamped) >= fabsf(_v) ? _cfg.aMax : _cfg.aDecel) * dt_s;
        _v = approach(_v, vTgtClamped, dv_max);
    }

    // ------------------------------------------------------------------
    // Yaw channel — symmetric trapezoid with optional jerk limit.
    //
    // yawRateMax and yawAccMax are stored in deg/s and deg/s²; convert
    // to rad/s and rad/s² at the use site.
    //
    // At yawJerkMax == 0 (default): pure trapezoid (identical to pre-018).
    // At yawJerkMax > 0: S-curve on omega via _omegaALive.
    // ------------------------------------------------------------------
    float yawRateMax_rad = _cfg.yawRateMax * kDegToRad;
    float yawAccMax_rad  = _cfg.yawAccMax  * kDegToRad;

    float omegaTgtClamped = clamp(_omegaTgt, -yawRateMax_rad, +yawRateMax_rad);

    if (_cfg.yawJerkMax > 0.0f) {
        // S-curve path for yaw.  Same approach-based integration as linear
        // channel: prevents overshoot while preserving jerk-limited ramp.
        float yawJerkMaxRad = _cfg.yawJerkMax * kDegToRad;
        float omegaATarget  = (omegaTgtClamped >= _omega) ? +yawAccMax_rad : -yawAccMax_rad;
        _omegaALive = approach(_omegaALive, omegaATarget, yawJerkMaxRad * dt_s);
        _omega = approach(_omega, omegaTgtClamped, fabsf(_omegaALive * dt_s));
    } else {
        // Trapezoid path (yawJerkMax == 0): identical to pre-018 behaviour.
        float domega_max = yawAccMax_rad * dt_s;
        _omega = approach(_omega, omegaTgtClamped, domega_max);
    }

    // ------------------------------------------------------------------
    // Per-tick ordering invariant:
    //   profile → inverse → saturate → setTarget
    // ------------------------------------------------------------------
    float vL, vR, sL, sR;
    BodyKinematics::inverse(_v, _omega, _cfg.trackwidthMm, vL, vR);
    BodyKinematics::saturate(vL, vR, _cfg.vWheelMax, _cfg.steerHeadroom, sL, sR);
    _mc.setTarget(sL, sR);

    return !atTarget();
}

// ---------------------------------------------------------------------------
// Reset / seed
// ---------------------------------------------------------------------------

void BodyVelocityController::reset()
{
    _v         = 0.0f;
    _omega     = 0.0f;
    _vTgt      = 0.0f;
    _omegaTgt  = 0.0f;
    _aLive     = 0.0f;
    _omegaALive = 0.0f;
}

void BodyVelocityController::seedCurrent(float v_mms, float omega_rads)
{
    _v     = v_mms;
    _omega = omega_rads;
}

// ---------------------------------------------------------------------------
// Convergence test
// ---------------------------------------------------------------------------

bool BodyVelocityController::atTarget() const
{
    float vTgtClamped     = clamp(_vTgt, -_cfg.vBodyMax, +_cfg.vBodyMax);
    float yawRateMax_rad  = _cfg.yawRateMax * kDegToRad;
    float omegaTgtClamped = clamp(_omegaTgt, -yawRateMax_rad, +yawRateMax_rad);

    return (fabsf(_v     - vTgtClamped)     < 0.5f) &&
           (fabsf(_omega - omegaTgtClamped) < 0.001f);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

float BodyVelocityController::approach(float cur, float tgt, float step)
{
    float delta = tgt - cur;
    if (delta > +step) delta = +step;
    if (delta < -step) delta = -step;
    return cur + delta;
}

float BodyVelocityController::clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
