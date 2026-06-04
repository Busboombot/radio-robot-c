#include "Odometry.h"
#include <math.h>

Odometry::Odometry()
    : _prevEncL(0.0f), _prevEncR(0.0f)
    , _otosRejected(0)
{
}

// ---------------------------------------------------------------------------
// predict — midpoint (exact-arc) integration (docs/kinematics-model.md §2.4)
//
// Reads s.encLMm / s.encRMm; writes s.poseX / s.poseY / s.poseHrad.
// ---------------------------------------------------------------------------

void Odometry::predict(HardwareState& s, float trackwidthMm)
{
    float dL = s.encLMm - _prevEncL;
    float dR = s.encRMm - _prevEncR;
    _prevEncL = s.encLMm;
    _prevEncR = s.encRMm;

    float dCenter   = (dL + dR) * 0.5f;
    float dTheta    = (dR - dL) / trackwidthMm;
    float thetaMid  = s.poseHrad + dTheta * 0.5f;

    s.poseX    += dCenter * cosf(thetaMid);
    s.poseY    += dCenter * sinf(thetaMid);
    s.poseHrad  = wrapPi(s.poseHrad + dTheta);
}

// ---------------------------------------------------------------------------
// correct — OTOS complementary correction (docs/kinematics-model.md §2.4)
//
// Reads and writes s.poseX / s.poseY / s.poseHrad.
// ---------------------------------------------------------------------------

void Odometry::correct(HardwareState& s,
                       float x_otos, float y_otos, float theta_otos_rad,
                       float alphaPos, float alphaYaw, float otosGate)
{
    // Outlier gate: reject if OTOS position disagrees with predicted pose
    // by more than the gate threshold.
    float dx = x_otos - s.poseX;
    float dy = y_otos - s.poseY;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist > otosGate) {
        ++_otosRejected;
        return;
    }

    // Accepted: complementary blend of position.
    s.poseX += alphaPos * dx;
    s.poseY += alphaPos * dy;

    // Heading blend: angle-wrap-safe — blend on the angular difference,
    // not on the raw angle, to avoid crossing the ±π discontinuity.
    float dh = wrapPi(theta_otos_rad - s.poseHrad);
    s.poseHrad = wrapPi(s.poseHrad + alphaYaw * dh);
}

// ---------------------------------------------------------------------------
// getPose — read pose from s and convert to integer mm + centidegrees.
// ---------------------------------------------------------------------------

void Odometry::getPose(const HardwareState& s,
                       int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg)
{
    x_mm = static_cast<int32_t>(s.poseX);
    y_mm = static_cast<int32_t>(s.poseY);

    float cdeg = s.poseHrad * RAD_TO_CDEG;
    if (cdeg >  18000.0f) cdeg =  18000.0f;
    if (cdeg < -18000.0f) cdeg = -18000.0f;
    h_cdeg = static_cast<int32_t>(cdeg);
}

// ---------------------------------------------------------------------------
// setPose — write pose into s; also reset prev-encoder snapshot.
// ---------------------------------------------------------------------------

void Odometry::setPose(HardwareState& s, int32_t x_mm, int32_t y_mm, int32_t h_cdeg)
{
    s.poseX    = static_cast<float>(x_mm);
    s.poseY    = static_cast<float>(y_mm);
    s.poseHrad = static_cast<float>(h_cdeg) * CDEG_TO_RAD;
    _prevEncL  = 0.0f;
    _prevEncR  = 0.0f;
}

// ---------------------------------------------------------------------------
// zero — reset pose to origin; reset prev-encoder snapshot.
// ---------------------------------------------------------------------------

void Odometry::zero(HardwareState& s)
{
    setPose(s, 0, 0, 0);
}

// ---------------------------------------------------------------------------
// update — legacy forward-Euler (deprecated; callers should use predict()).
// ---------------------------------------------------------------------------

void Odometry::update(HardwareState& s, float dL_mm, float dR_mm, float trackwidthMm)
{
    float dCenter = (dL_mm + dR_mm) * 0.5f;
    float dTheta  = (dR_mm - dL_mm) / trackwidthMm;

    s.poseX    += dCenter * cosf(s.poseHrad);
    s.poseY    += dCenter * sinf(s.poseHrad);
    s.poseHrad += dTheta;
}

// ---------------------------------------------------------------------------
// wrapPi — keep heading in (-π, π]
// ---------------------------------------------------------------------------

float Odometry::wrapPi(float theta)
{
    return atan2f(sinf(theta), cosf(theta));
}
