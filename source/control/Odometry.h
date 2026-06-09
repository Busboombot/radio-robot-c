#pragma once
#include <stdint.h>
#include "RobotState.h"
#include "CommandTypes.h"
#include "OtosSensor.h"

// Forward-declare Odometry so OdomCtx can hold a pointer to it.
// The full class definition follows immediately below.
class Odometry;

/**
 * OdomCtx — context bundle for Odometry Commandable handlers.
 *
 * Both pointers are populated by Odometry::setCtx() before any command
 * can arrive.  All seven OTOS command handlers (OI, OZ, OR, OP, OV, OL, OA)
 * reach the OtosSensor through this struct.
 */
struct OdomCtx {
    Odometry*   odo;
    OtosSensor* otos;
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
 */
class Odometry : public Commandable {
public:
    Odometry();

    // Commandable interface — returns descriptors for all 7 OTOS commands.
    virtual int getCommands(CommandDescriptor* buf, int max) const override;

    // Bind the OtosSensor so command handlers can reach it.
    // Called by Robot after construction.
    void setCtx(OtosSensor* otos) { _odomCtx.odo = this; _odomCtx.otos = otos; }

    // ---------------------------------------------------------------------------
    // Primary API — struct-based (014-004)
    // ---------------------------------------------------------------------------

    // Midpoint (exact-arc) integration — primary predict step.
    // Reads s.encLMm / s.encRMm, computes deltas against _prevEncL/_prevEncR,
    // then writes the updated pose into s.poseX / s.poseY / s.poseHrad.
    void predict(HardwareState& s, float trackwidthMm);

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
    // Also resets _prevEncL/_prevEncR to 0 so the next predict() uses a
    // fresh encoder snapshot.
    void setPose(HardwareState& s, int32_t x_mm, int32_t y_mm, int32_t h_cdeg);

    // Zero pose in s: equivalent to setPose(s, 0, 0, 0).
    void zero(HardwareState& s);

    // ---------------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------------

    // Number of OTOS samples rejected by the outlier gate since boot.
    uint32_t otosRejectedCount() const { return _otosRejected; }

    // ---------------------------------------------------------------------------
    // Legacy forward-Euler integrate (deprecated; kept for reference only).
    // dL_mm, dR_mm: signed mm traveled by left and right wheels this tick.
    // NOTE: this overload writes to a local state — do not use in new code.
    // ---------------------------------------------------------------------------
    void update(HardwareState& s, float dL_mm, float dR_mm, float trackwidthMm);

private:
    // Intermediate compute state: previous encoder snapshot (not in HardwareState
    // because Odometry runs at a different cadence than the control task).
    float    _prevEncL;   // last encoder snapshot, mm
    float    _prevEncR;   // last encoder snapshot, mm

    uint32_t _otosRejected; // count of OTOS samples rejected by outlier gate

    mutable OdomCtx _odomCtx; // context bundle for Commandable handlers

    // Wrap heading to (-π, π] using atan2f identity.
    static float wrapPi(float theta);

    static constexpr float PI_F        = 3.14159265f;
    static constexpr float RAD_TO_CDEG = 18000.0f / 3.14159265f;
    static constexpr float CDEG_TO_RAD = 3.14159265f / 18000.0f;
};
