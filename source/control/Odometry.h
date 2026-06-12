#pragma once
#include <stdint.h>
#include "RobotState.h"
#include "CommandTypes.h"
#include "IOtosSensor.h"
#include "EKF.h"

// ---------------------------------------------------------------------------
// effectiveSlip — migration-safe rotationalSlip clamp.
//
// Used by Odometry::predict() and MotionController::beginRotation() so both
// apply the same clamped slip factor from RobotConfig::rotationalSlip.
//
// Value semantics:
//   0.0 (or negative, or unset) → 1.0  (no correction; legacy config-safe)
//   (0.0, 0.5)                  → 0.5  (clamp floor — unrealistic slip)
//   [0.5, 1.0]                  → pass-through
//   > 1.0                       → 1.0  (clamp ceiling)
//
// Sprint 024-006: rotationalSlip is now wired into firmware logic (was dead).
// ---------------------------------------------------------------------------
inline float effectiveSlip(float rawSlip) {
    if (rawSlip <= 0.0f) return 1.0f;
    if (rawSlip < 0.5f)  return 0.5f;
    if (rawSlip > 1.0f)  return 1.0f;
    return rawSlip;
}

// Forward-declare Odometry so OdomCtx can hold a pointer to it.
// The full class definition follows immediately below.
class Odometry;

/**
 * OdomCtx — context bundle for Odometry Commandable handlers.
 *
 * All pointers are populated by Odometry::setCtx() before any command
 * can arrive.  OTOS command handlers (OI, OZ, OR, OV, OL, OA) reach the
 * OtosSensor through this struct.  handleOP reads hwState directly (cached
 * state from the main loop) instead of calling otos->getPositionRaw().
 */
struct OdomCtx {
    Odometry*            odo;
    IOtosSensor*         otos;
    const HardwareState* hwState;  // cached OTOS pose for OP read (no device call)
};

/**
 * Odometry — differential-drive dead-reckoning pose tracker.
 *
 * Heading convention: 0 = +X axis, positive = CCW (standard math).
 * Output convention: centidegrees (360 degrees = 36000 cdeg).
 *
 * Authoritative pose (poseX, poseY, poseHrad) lives in HardwareState.
 * Odometry owns only its previous-encoder snapshot (_prevEncL/_prevEncR)
 * as intermediate compute state — separate from the control task's counters.
 *
 * Primary API: call predict(HardwareState&, trackwidthMm) once per
 * odometry-predict task tick.  Reads encLMm/R from the struct, computes
 * deltas against _prevEncL/_prevEncR, applies midpoint (exact-arc)
 * integration per docs/kinematics-model.md §2.4, and writes poseX/Y/Hrad
 * back into the struct:
 *
 *   dC = (dL + dR)/2 ;  dθ = (dR − dL)/b
 *   θ_mid = θ + dθ/2
 *   x += dC·cos(θ_mid) ;  y += dC·sin(θ_mid) ;  θ = wrapπ(θ + dθ)
 *
 * Sprint 010, Ticket 005.
 * Sprint 010, Ticket 006: adds correct() for OTOS complementary fusion.
 * Sprint 014, Ticket 004: pose moved to HardwareState; struct-based API.
 * Sprint 022, Ticket 003: EKF integration — predict() drives EKF each tick;
 *   initEKF()/correctEKF() wire noise params and OTOS position updates.
 * Sprint 023, Ticket 003: 5-state EKF wiring — initEKF() extended with
 *   velocity noise params; predict() gains now_ms for dt computation;
 *   correctEKF() extended for OTOS + encoder velocity fusion. setPose
 *   re-baseline fix (_prevEncL = s.encLMm, not 0) prevents spurious
 *   encoder-delta jumps after camera fixes.
 */
class Odometry : public Commandable {
public:
    Odometry();

    virtual std::vector<CommandDescriptor> getCommands() const override;

    // Bind the OtosSensor and cached HardwareState so command handlers can reach them.
    // Called by Robot after construction.  hwState may be nullptr in unit tests
    // that do not exercise OP; handleOP checks for null before dereferencing.
    void setCtx(IOtosSensor* otos, const HardwareState* hwState = nullptr) {
        _odomCtx.odo     = this;
        _odomCtx.otos    = otos;
        _odomCtx.hwState = hwState;
    }

    // ---------------------------------------------------------------------------
    // Primary API — struct-based (014-004)
    // ---------------------------------------------------------------------------

    // Midpoint (exact-arc) integration — primary predict step.
    // Reads s.encLMm / s.encRMm, computes deltas against _prevEncL/_prevEncR,
    // then writes the updated pose into s.poseX / s.poseY / s.poseHrad.
    // Also advances the EKF and writes EKF state as the authoritative pose,
    // including s.fusedV and s.fusedOmega from the velocity states.
    //
    // rotationalSlip: body-rotation efficiency from RobotConfig (024-006).
    //   dTheta = ((dR-dL)/trackwidth) * clamp(rotationalSlip, 0.5, 1.0).
    //   0 / unset → 1.0 (migration-safe: no change for legacy exact-profile configs).
    // now_ms: robot system clock timestamp (ms). Used to compute dt for the
    // EKF and encoder-rate velocity. Signed delta cast avoids uint32 underflow
    // (see watchdog-uint32-underflow finding).
    void predict(HardwareState& s, float trackwidthMm,
                 float rotationalSlip, uint32_t now_ms);

    // EKF initialisation — set process and measurement noise parameters.
    // Must be called once at startup (e.g. from Robot constructor) before
    // predict() or correctEKF() are called.
    //   q_xy         — process noise variance for x and y (mm^2)
    //   q_theta      — process noise variance for heading (rad^2)
    //   q_v          — process noise variance for linear velocity (mm/s)^2
    //   q_omega      — process noise variance for angular velocity (rad/s)^2
    //   r_otos_xy    — OTOS position measurement noise variance (mm^2)
    //   r_otos_v     — OTOS velocity measurement noise variance ((mm/s)^2)
    //   r_enc_v      — encoder velocity measurement noise variance ((mm/s)^2)
    //   r_otos_theta — OTOS heading measurement noise variance (rad^2)
    void initEKF(float q_xy, float q_theta, float q_v, float q_omega,
                 float r_otos_xy, float r_otos_v, float r_enc_v,
                 float r_otos_theta);

    // EKF correction — fuse OTOS position, heading, and velocity measurements,
    // plus encoder-derived velocity, into the 5-state EKF.
    // Call order: updatePosition → updateHeading → updateVelocity(OTOS) →
    //             updateVelocity(enc). All channels apply Mahalanobis gating
    // internally. Writes all EKF outputs back to s.
    //   x_otos, y_otos         — OTOS position observation (mm)
    //   theta_otos_rad         — OTOS heading observation (rad) — sprint 024-004
    //   v_otos_mmps            — OTOS body-frame linear velocity (mm/s)
    //   omega_otos_rads        — OTOS angular velocity (rad/s)
    //   v_enc_mmps             — encoder-derived linear velocity (mm/s)
    //   omega_enc_rads         — encoder-derived angular velocity (rad/s)
    void correctEKF(HardwareState& s,
                    float x_otos, float y_otos,
                    float theta_otos_rad,
                    float v_otos_mmps, float omega_otos_rads,
                    float v_enc_mmps, float omega_enc_rads);

    // OTOS complementary correction — correct step of predict/correct.
    // (docs/kinematics-model.md §2.4; EKF upgrade path replaces this later.)
    //
    // Parameters x_otos, y_otos, theta_otos_rad are passed by the caller
    // (who reads them from s.otosX/Y/H after unit conversion) for testability.
    //
    // Outlier gate: if distance(otos, predicted pose) > otosGate, the sample
    // is rejected; _otosRejected is incremented and s.poseX/Y/Hrad are unchanged.
    //
    // If accepted:
    //   s.poseX    += alphaPos * (x_otos - s.poseX)
    //   s.poseY    += alphaPos * (y_otos - s.poseY)
    //   s.poseHrad += alphaYaw * wrapπ(θ_otos - s.poseHrad)   [angle-wrap safe]
    //
    // alphaPos, alphaYaw: blend gains in [0, 1] (from RobotConfig).
    // otosGate: rejection distance threshold in mm (from RobotConfig).
    void correct(HardwareState& s,
                 float x_otos, float y_otos, float theta_otos_rad,
                 float alphaPos, float alphaYaw, float otosGate);

    // ---------------------------------------------------------------------------
    // Pose read / write helpers — operate on a HardwareState reference.
    // ---------------------------------------------------------------------------

    // Read current pose from s.  x_mm and y_mm are integer mm; h_cdeg is
    // centidegrees (-18000..+18000 clamped).
    static void getPose(const HardwareState& s,
                        int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg);

    // Overwrite pose in s (used by Robot::distanceDrive / SI command).
    // h_cdeg is centidegrees; stored internally as radians.
    // Re-baselines _prevEncL/_prevEncR to s.encLMm/s.encRMm (not 0) so the
    // next predict() sees a delta of ~0 instead of a spurious jump.
    // (Sprint 023: was = 0.0f, which caused encoder-delta corruption after
    // every camera fix when encoders were non-zero.)
    void setPose(HardwareState& s, int32_t x_mm, int32_t y_mm, int32_t h_cdeg);

    // Zero pose in s: equivalent to setPose(s, 0, 0, 0).
    void zero(HardwareState& s);

    // rebaselinePrev — reset the internal encoder snapshot to (encL, encR) without
    // touching pose.  Called by Robot::resetEncoders() so that the next predict()
    // sees a delta of 0 after a hardware accumulator reset, preventing the
    // large-negative-delta backward teleport on every D command and ZERO enc.
    // (N1 fix, sprint 030-001.)
    void rebaselinePrev(float encL, float encR) {
        _prevEncL = encL;
        _prevEncR = encR;
    }

    // ---------------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------------

    // Number of OTOS samples rejected by the outlier gate since boot.
    uint32_t otosRejectedCount() const { return _otosRejected; }

    // Cumulative EKF gate rejection count (all channels: position, heading, velocity).
    // Sprint 024-005: exposed for TLM ekf_rej= field.
    int ekfRejectCount() const { return _ekf.getRejectCount(); }

    // ---------------------------------------------------------------------------
    // Legacy forward-Euler integrate (deprecated; kept for reference only).
    // dL_mm, dR_mm: signed mm traveled by left and right wheels this tick.
    // NOTE: this overload writes to a local state — do not use in new code.
    // ---------------------------------------------------------------------------
    void update(HardwareState& s, float dL_mm, float dR_mm, float trackwidthMm);

    // Encoder-rate velocity from the most recent predict() call.
    // Stored so correctEKF() (called from otosCorrect()) can pass them to the
    // EKF velocity update channels without changing the cooperative loop's
    // call signature.  Zero-initialised; updated each predict() tick.
    //
    // Design choice: store on Odometry rather than pass through the loop caller
    // because predict() and otosCorrect() run on different loop phases (enOdom
    // vs enOtos), so threading enc_v through the caller would require storing
    // them in HardwareState or Robot anyway — no fewer coupling points.
    float lastEncV()     const { return _lastEncV; }     // body linear speed, mm/s
    float lastEncOmega() const { return _lastEncOmega; } // yaw rate, rad/s

private:
    // Intermediate compute state: previous encoder snapshot (not in HardwareState
    // because Odometry runs at a different cadence than the control task).
    float    _prevEncL;   // last encoder snapshot, mm
    float    _prevEncR;   // last encoder snapshot, mm

    uint32_t _otosRejected; // count of OTOS samples rejected by outlier gate

    // dt tracking for encoder-rate velocity and EKF timestep.
    // uint32_t but delta is cast to int32_t before arithmetic to avoid underflow.
    uint32_t _lastPredictMs;  // timestamp of last predict() call (0 = not yet called)

    // Noise params stored from initEKF() — passed to update*() methods.
    // Using _rOtosV for both v and omega of OTOS source (symmetric simplification),
    // and _rEncV for both v and omega of encoder source.
    float _rOtosV;     // OTOS velocity measurement noise variance ((mm/s)^2)
    float _rEncV;      // encoder velocity measurement noise variance ((mm/s)^2)
    float _rOtosTheta; // OTOS heading measurement noise variance (rad^2) — sprint 024-004

    // Encoder-derived velocity from the most recent predict() call.
    // Set to 0 until the first valid predict() tick (dt > 0).
    float _lastEncV;     // body linear speed (mm/s)
    float _lastEncOmega; // yaw rate (rad/s)

    EKF _ekf;              // Extended Kalman Filter — fuses encoder odometry with OTOS

    mutable OdomCtx _odomCtx; // context bundle for Commandable handlers

    // Wrap heading to (-π, π] using atan2f identity.
    static float wrapPi(float theta);

    static constexpr float PI_F        = 3.14159265f;
    static constexpr float RAD_TO_CDEG = 18000.0f / 3.14159265f;
    static constexpr float CDEG_TO_RAD = 3.14159265f / 18000.0f;
};
