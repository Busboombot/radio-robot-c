#pragma once
#include <stdint.h>
#include "IOtosSensor.h"

struct RobotConfig;

/**
 * BenchOtosSensor — IOtosSensor implementation for bench testing on a robot stand.
 *
 * The real OtosSensor sees no motion when the robot is on a stand.  This class
 * synthesises plausible OTOS output by integrating COMMANDED wheel velocity into
 * an ideal pose, then optionally adding sensor noise and slow yaw drift.  It lets
 * the full firmware stack (EKF, distance stops, TLM) run on the bench without I2C.
 *
 * Two independent accumulators:
 *   _idealX/Y/H  — noiseless arc-integration (ground truth for DBG OTOS BENCH)
 *   _otosX/Y/H   — errored integration (what readTransformed() returns)
 *
 * Error model (applied per tick):
 *   - Gaussian noise on the arc-distance dC  (sigma = _noiseXY mm/mm of motion)
 *   - Gaussian noise on the heading change dTh (sigma = _noiseH rad/rad of turn)
 *   - Slow yaw drift: _driftRadPerSec * dt_s added to _otosH each tick
 *
 * PRNG:
 *   HOST_BUILD — deterministic Box-Muller with a fixed-seed LCG (reproducible).
 *   Firmware   — microbit_random(int max) approximation via sum-of-uniforms
 *                (central-limit Gaussian; no std:: needed).
 *
 * This class lives in source/hal/ and is compiled into the firmware build
 * automatically.  The host build includes it explicitly via host_tests/CMakeLists.txt.
 *
 * No I2C dependency.  begin() always succeeds; is_initialized() returns true.
 */
class BenchOtosSensor : public IOtosSensor {
public:
    BenchOtosSensor();

    // -------------------------------------------------------------------------
    // IOtosSensor interface
    // -------------------------------------------------------------------------

    // Returns the current errored pose (_otosX/Y/H) in poseOut.
    // Always returns true (bench sensor is always valid).
    // cfg and headingRad are ignored (no lever-arm correction in bench mode).
    bool readTransformed(const RobotConfig& cfg, OtosPose& poseOut,
                         float headingRad = 0.0f) const override;

    // Returns body-frame velocity derived from the last tick's errored arc step.
    // Always returns true.
    bool readVelocityTransformed(const RobotConfig& cfg, OtosVelocity& velOut,
                                 float headingRad = 0.0f) const override;

    // Returns true; out = 0 (always valid, never lifted).
    bool readStatus(uint8_t& out) const override;

    // Always returns true.
    bool lastReadOk() const override;

    // Returns {0, 0} — no accelerometer in bench mode.
    OtosAccel readAccelTransformed(const RobotConfig& cfg) const override;

    // Calibration stubs — all no-ops; get* return 0.
    void   init() override {}
    void   calibrateImu(uint8_t samples) override { (void)samples; }
    void   resetTracking() override {}

    void   getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const override;
    void   setPositionRaw(int16_t x, int16_t y, int16_t h) override;

    int8_t getLinearScalar()         const override { return 0; }
    void   setLinearScalar(int8_t)         override {}
    int8_t getAngularScalar()        const override { return 0; }
    void   setAngularScalar(int8_t)        override {}

    // -------------------------------------------------------------------------
    // Sensor interface
    // -------------------------------------------------------------------------
    // Sets _initialized = true; always returns true (no I2C probe).
    bool begin() override;

    // -------------------------------------------------------------------------
    // Bench-specific interface (not on IOtosSensor)
    // -------------------------------------------------------------------------

    /**
     * tick — integrate one control step into both accumulators.
     *
     * velLMms, velRMms : commanded left/right wheel velocities, mm/s.
     * trackwidthMm     : wheel-to-wheel track width, mm.
     * dt_ms            : elapsed time for this step, ms.
     *
     * No-op when dt_ms == 0, trackwidthMm <= 0, or is_initialized() is false.
     */
    void tick(float velLMms, float velRMms, float trackwidthMm, uint32_t dt_ms);

    /**
     * enable / enabled — gate whether tick() advances the accumulators.
     * When disabled the sensor is still initialized and returns the last pose.
     */
    void enable(bool on) { _enabled = on; }
    bool enabled()  const { return _enabled; }

    /**
     * setNoise — update the error model parameters at runtime.
     *
     * noiseXY       : per-tick linear noise sigma (fraction of arc distance).
     * noiseH        : per-tick yaw noise sigma (fraction of heading change).
     * driftRadPerSec: slow additive yaw drift added to _otosH every second.
     */
    void setNoise(float noiseXY, float noiseH, float driftRadPerSec);

    /** idealPose — fill out with the noiseless accumulator for DBG OTOS queries. */
    void idealPose(OtosPose& out) const;

    /** reset — zero both accumulators and the velocity/accel state. */
    void reset();

    // Noiseless accumulator accessors (convenience for tests).
    float idealX() const { return _idealX; }
    float idealY() const { return _idealY; }
    float idealH() const { return _idealH; }

    // Errored accumulator accessors (convenience for tests).
    float otosX() const { return _otosX; }
    float otosY() const { return _otosY; }
    float otosH() const { return _otosH; }

private:
    // Noiseless accumulator (ground truth).
    float _idealX = 0.0f;
    float _idealY = 0.0f;
    float _idealH = 0.0f;

    // Errored accumulator (what readTransformed returns).
    float _otosX  = 0.0f;
    float _otosY  = 0.0f;
    float _otosH  = 0.0f;

    // Error model parameters.
    float _noiseXY       = 0.0f;  // linear noise sigma (fraction of dC)
    float _noiseH        = 0.0f;  // yaw noise sigma (fraction of dTh)
    float _driftRadPerSec = 0.0f; // slow additive yaw drift, rad/s

    // Last-tick body-frame velocity/accel (updated by tick, returned by readVelocityTransformed).
    float _velV      = 0.0f;  // forward speed, mm/s
    float _velOmega  = 0.0f;  // yaw rate, rad/s
    float _accAx     = 0.0f;  // forward accel, mm/s^2
    float _prevVelV  = 0.0f;  // for finite-difference accel

    bool _enabled = true;

    // HOST_BUILD PRNG state — deterministic LCG for reproducible simulation.
#ifdef HOST_BUILD
    mutable uint32_t _lcgState = 12345u;
    // Advance LCG and return a uniform float in [0, 1).
    float lcgRand() const;
    // Box-Muller normal variate with zero mean, given sigma.
    float gaussRand(float sigma) const;
#else
    // Firmware PRNG — sum-of-uniforms approximation via microbit_random.
    float gaussRandFW(float sigma) const;
#endif

    // Wrap angle to [-pi, pi].
    static float wrapAngle(float a);
};
