#pragma once
#include "Config.h"

class MotorController;

/**
 * BodyVelocityController — body-level (v, ω) motion profiler.
 *
 * Ramps the live body twist (v, ω) toward a commanded target under
 * configurable acceleration and rate limits, then translates to wheel
 * setpoints each tick via BodyKinematics::inverse → saturate →
 * MotorController::setTarget.
 *
 * Implements the Layer-2 body profiler defined in:
 *   .clasi/sprints/017-.../architecture-update.md  §BodyVelocityController
 *
 * Per-tick ordering invariant (must not be reordered):
 *   1. Trapezoid profile step (approach v and omega under limits).
 *   2. BodyKinematics::inverse(v, omega, trackwidthMm, vL, vR).
 *   3. BodyKinematics::saturate(vL, vR, vWheelMax, steerHeadroom, sL, sR).
 *   4. MotorController::setTarget(sL, sR).
 *
 * Clock discipline: advance(dt_s) is ticked exactly once per control tick
 * by the owner (MotionCommand::tick, called from MotionController::driveAdvance).
 * It must NOT be called from any other path — double-advancing the profiler
 * double-counts the ramp step and over-accelerates the robot.
 *
 * Rate limits are read live from const RobotConfig& each tick so that SET
 * commands take effect immediately without requiring a restart.
 *
 * Thread safety: single-threaded tick loop only.
 */
class BodyVelocityController {
public:
    /**
     * Construct, storing references to the motor controller and config.
     * All profiler state is initialised to zero.
     *
     * @param mc   Motor controller to write wheel setpoints into.
     * @param cfg  Robot config providing live acceleration/rate limits.
     */
    BodyVelocityController(MotorController& mc, const RobotConfig& cfg);

    /**
     * setTarget — update the commanded body twist.
     *
     * Does not step the profiler; call advance() to ramp toward the new target.
     *
     * @param v_mms      Desired body forward speed, mm/s (signed; forward > 0).
     * @param omega_rads Desired yaw rate, rad/s (CCW-positive).
     */
    void setTarget(float v_mms, float omega_rads);

    /**
     * advance — step the profiler one control tick.
     *
     * Reads live limits from cfg each call.  After clamping and ramping,
     * calls BodyKinematics::inverse → saturate → mc.setTarget.
     *
     * When cfg.jMax > 0, uses the S-curve (jerk-limited) path: slews the
     * live acceleration toward the demanded step under the jerk bound, then
     * integrates velocity.  At cfg.jMax == 0 (default), degenerates to the
     * pure trapezoid.  Same logic applies to the yaw channel via yawJerkMax.
     *
     * Must be called exactly once per driveAdvance tick (via MotionCommand::tick).
     *
     * @param dt_s  Elapsed time since the previous advance call, seconds.
     *              Must be > 0; values <= 0 are treated as a no-op.
     * @return      true while still ramping toward target; false when atTarget().
     */
    bool advance(float dt_s);

    /**
     * reset — zero the profiler state (_v, _omega, _vTgt, _omegaTgt,
     *         _aLive, _omegaALive).
     *
     * Does NOT call MotorController::stop() — the caller is responsible for
     * deciding whether to coast or brake.
     */
    void reset();

    /**
     * seedCurrent — set the live profiler state to the given values.
     *
     * Use when handing off from another mode so the next advance() ramps
     * from the current actual twist rather than from zero (avoids a lurch).
     *
     * @param v_mms      Current body forward speed, mm/s.
     * @param omega_rads Current yaw rate, rad/s.
     */
    void seedCurrent(float v_mms, float omega_rads);

    /** currentV — live profiled body forward speed, mm/s. */
    float currentV()     const { return _v; }

    /** currentOmega — live profiled yaw rate, rad/s. */
    float currentOmega() const { return _omega; }

    /** targetV — commanded body forward speed (before clamping), mm/s. */
    float targetV()      const { return _vTgt; }

    /** targetOmega — commanded yaw rate (before clamping), rad/s. */
    float targetOmega()  const { return _omegaTgt; }

    /**
     * atTarget — returns true when the profiler has converged on the target.
     *
     * Convergence thresholds: |v - vTgt_clamped| < 0.5 mm/s and
     *                         |omega - omegaTgt_clamped| < 0.001 rad/s.
     * The clamped targets are re-derived from the live cfg limits.
     */
    bool atTarget() const;

private:
    MotorController&   _mc;
    const RobotConfig& _cfg;

    float _v;           // live profiled body forward speed, mm/s
    float _omega;       // live profiled yaw rate, rad/s
    float _vTgt;        // commanded forward speed (caller-supplied), mm/s
    float _omegaTgt;    // commanded yaw rate (caller-supplied), rad/s
    float _aLive;       // current live linear acceleration, mm/s² (S-curve channel)
    float _omegaALive;  // current live yaw acceleration, rad/s²   (S-curve channel)

    /**
     * approach — single-axis trapezoid step helper.
     *
     * Advances cur toward tgt by at most step in either direction.
     * Equivalent to: cur + clamp(tgt - cur, -step, +step).
     */
    static float approach(float cur, float tgt, float step);

    /** clamp — symmetric clamp helper. */
    static float clamp(float v, float lo, float hi);
};
