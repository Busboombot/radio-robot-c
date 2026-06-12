#include "BenchOtosSensor.h"
#include <cmath>

// M_PI guard — micro:bit / ARMCC may not define M_PI by default.
// Match the pattern used by StopCondition.cpp and BodyVelocityController.cpp.
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ---------------------------------------------------------------------------
// PRNG — two implementations selected at compile time.
//
// HOST_BUILD: deterministic LCG + Box-Muller for reproducible simulation.
// Firmware:   sum-of-uniforms (central-limit theorem) using microbit_random.
// ---------------------------------------------------------------------------

#ifdef HOST_BUILD
// No CODAL headers in host builds; use a deterministic LCG for reproducibility.

float BenchOtosSensor::lcgRand() const {
    // LCG parameters from Knuth: multiplier 1664525, addend 1013904223.
    _lcgState = _lcgState * 1664525u + 1013904223u;
    // Convert to [0, 1) by dividing by 2^32.
    return static_cast<float>(_lcgState) / 4294967296.0f;
}

float BenchOtosSensor::gaussRand(float sigma) const {
    if (sigma <= 0.0f) return 0.0f;
    // Box-Muller transform: two uniforms U1, U2 → standard normal Z.
    // Clamp U1 away from zero to avoid log(0).
    float u1 = lcgRand();
    if (u1 < 1e-10f) u1 = 1e-10f;
    float u2 = lcgRand();
    // Cosine branch only (one variate per call).
    float z = sqrtf(-2.0f * logf(u1)) *
              cosf(2.0f * static_cast<float>(M_PI) * u2);
    return z * sigma;
}

#else
// Firmware build — use CODAL microbit_random().
#include "MicroBitDevice.h"

// Sum-of-uniforms Gaussian approximation via the central limit theorem.
// microbit_random(N) returns an int in [0, N-1].
// Sum 6 uniform[-1,1] samples → approximately Normal(0, sqrt(2)).
// Scale by sigma / sqrt(2) to achieve the target sigma.
float BenchOtosSensor::gaussRandFW(float sigma) const {
    if (sigma <= 0.0f) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < 6; ++i) {
        float u = static_cast<float>(microbit_random(10001)) / 5000.0f - 1.0f;
        sum += u;
    }
    // sum ∈ [-6, 6], std dev ≈ sqrt(2); scale to unit normal then to sigma.
    static const float INV_SQRT2 = 0.70710678f;
    return sum * INV_SQRT2 * sigma;
}
#endif // HOST_BUILD

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static const float PI_F = static_cast<float>(M_PI);

float BenchOtosSensor::wrapAngle(float a) {
    while (a >  PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

// ---------------------------------------------------------------------------
// Constructor / begin / reset
// ---------------------------------------------------------------------------

BenchOtosSensor::BenchOtosSensor() {}

bool BenchOtosSensor::begin() {
    _initialized = true;
    return true;
}

void BenchOtosSensor::reset() {
    _idealX   = _idealY   = _idealH   = 0.0f;
    _otosX    = _otosY    = _otosH    = 0.0f;
    _velV     = _velOmega = _accAx    = _prevVelV = 0.0f;
}

// ---------------------------------------------------------------------------
// IOtosSensor read methods
// ---------------------------------------------------------------------------

bool BenchOtosSensor::readTransformed(const RobotConfig& /*cfg*/, OtosPose& poseOut,
                                      float /*headingRad*/) const {
    poseOut.x = _otosX;
    poseOut.y = _otosY;
    poseOut.h = _otosH;
    return true;
}

bool BenchOtosSensor::readVelocityTransformed(const RobotConfig& /*cfg*/,
                                              OtosVelocity& velOut,
                                              float /*headingRad*/) const {
    velOut.v_mmps     = _velV;
    velOut.omega_rads = _velOmega;
    return true;
}

bool BenchOtosSensor::readStatus(uint8_t& out) const {
    out = 0;
    return true;
}

bool BenchOtosSensor::lastReadOk() const {
    return true;
}

OtosAccel BenchOtosSensor::readAccelTransformed(const RobotConfig& /*cfg*/) const {
    return { _accAx, 0.0f };
}

// ---------------------------------------------------------------------------
// Calibration stubs — raw position access
// ---------------------------------------------------------------------------

void BenchOtosSensor::getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const {
    x = 0; y = 0; h = 0;
}

void BenchOtosSensor::setPositionRaw(int16_t /*x*/, int16_t /*y*/, int16_t /*h*/) {}

// ---------------------------------------------------------------------------
// Error model
// ---------------------------------------------------------------------------

void BenchOtosSensor::setNoise(float noiseXY, float noiseH, float driftRadPerSec) {
    _noiseXY         = noiseXY;
    _noiseH          = noiseH;
    _driftRadPerSec  = driftRadPerSec;
}

// ---------------------------------------------------------------------------
// Ideal pose accessor
// ---------------------------------------------------------------------------

void BenchOtosSensor::idealPose(OtosPose& out) const {
    out.x = _idealX;
    out.y = _idealY;
    out.h = _idealH;
}

// ---------------------------------------------------------------------------
// tick — advance both accumulators one control step
// ---------------------------------------------------------------------------

void BenchOtosSensor::tick(float velLMms, float velRMms,
                           float trackwidthMm, uint32_t dt_ms) {
    if (!_initialized || !_enabled || dt_ms == 0 || trackwidthMm <= 0.0f) return;

    const float dt_s = static_cast<float>(dt_ms) / 1000.0f;

    // -- Ideal (noiseless) accumulator --
    {
        float dC   = (velLMms + velRMms) * 0.5f * dt_s;
        float dTh  = (velRMms - velLMms) / trackwidthMm * dt_s;
        float hMid = _idealH + dTh * 0.5f;
        _idealX   += dC * cosf(hMid);
        _idealY   += dC * sinf(hMid);
        _idealH    = wrapAngle(_idealH + dTh);
    }

    // -- Errored (noisy) accumulator --
    {
        float dC  = (velLMms + velRMms) * 0.5f * dt_s;
        float dTh = (velRMms - velLMms) / trackwidthMm * dt_s;

        // Per-step Gaussian noise on the arc segments.
#ifdef HOST_BUILD
        float noisyDC  = dC  + gaussRand(_noiseXY * fabsf(dC));
        float noisyDTh = dTh + gaussRand(_noiseH  * fabsf(dTh));
#else
        float noisyDC  = dC  + gaussRandFW(_noiseXY * fabsf(dC));
        float noisyDTh = dTh + gaussRandFW(_noiseH  * fabsf(dTh));
#endif

        float hMid  = _otosH + noisyDTh * 0.5f;
        _otosX     += noisyDC  * cosf(hMid);
        _otosY     += noisyDC  * sinf(hMid);
        _otosH      = wrapAngle(_otosH + noisyDTh + _driftRadPerSec * dt_s);

        // Body-frame velocity and finite-difference accel, consistent with
        // the position channel (derived from the same noisy arc segments).
        if (dt_s > 0.0f) {
            float newV     = noisyDC  / dt_s;
            float newOmega = noisyDTh / dt_s;
            _accAx     = (newV - _prevVelV) / dt_s;
            _prevVelV  = newV;
            _velV      = newV;
            _velOmega  = newOmega;
        }
    }
}
