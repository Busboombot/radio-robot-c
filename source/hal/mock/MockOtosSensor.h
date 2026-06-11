#pragma once
#include <stdint.h>
#include "../IOtosSensor.h"

#ifdef HOST_BUILD
#include <random>
#endif

struct RobotConfig;

/**
 * MockOtosSensor — host-compilable IOtosSensor implementation for unit tests.
 *
 * Returns zero pose by default. Tests inject a pose via setInjectedPose().
 * All calibration/raw-position methods are no-ops returning safe defaults.
 *
 * Sim model (optional): when enabled via enableSimModel(true), tick() integrates
 * true motor velocities with independent Gaussian noise into _odomX/Y/H.
 * readTransformed() returns the accumulated noisy pose when the sim model is on.
 *
 * No CODAL dependency. Compiles with plain clang++ -std=c++11 -I source.
 */
class MockOtosSensor : public IOtosSensor {
public:
    // IOtosSensor interface --------------------------------------------------
    OtosPose readTransformed(const RobotConfig& cfg) const override;
    OtosVelocity readVelocityTransformed(const RobotConfig& cfg) const override;
    OtosAccel readAccelTransformed(const RobotConfig& cfg) const override;

    void init() override {}
    void calibrateImu(uint8_t samples) override { (void)samples; }
    void resetTracking() override {}

    void getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const override;
    void setPositionRaw(int16_t x, int16_t y, int16_t h) override;

    int8_t getLinearScalar()  const override { return _linearScalar; }
    void   setLinearScalar(int8_t val) override { _linearScalar = val; }
    int8_t getAngularScalar() const override { return _angularScalar; }
    void   setAngularScalar(int8_t val) override { _angularScalar = val; }

    // Sensor interface -------------------------------------------------------
    bool begin() override { _initialized = true; return true; }

    // Test injection ---------------------------------------------------------
    void setInjectedPose(float x, float y, float h);

    float injectedX() const { return _injectedX; }
    float injectedY() const { return _injectedY; }
    float injectedH() const { return _injectedH; }

    // Sim model control ------------------------------------------------------
    void enableSimModel(bool on)          { _useSimModel = on; }
    void setLinearNoise(float sigma)      { _linearNoiseSigma = sigma; }
    void setYawNoise(float sigma)         { _yawNoiseSigma = sigma; }

    // Accumulated OTOS odometry (sim model output) ---------------------------
    float odomX() const { return _odomX; }
    float odomY() const { return _odomY; }
    float odomH() const { return _odomH; }

    // Advance the OTOS integration model by one time step.
    // velLMms, velRMms: true (pre-slip) left/right velocities in mm/s.
    // trackwidthMm: wheel-to-wheel distance in mm.
    // dt_ms: elapsed time since last tick.
    void tick(float velLMms, float velRMms, float trackwidthMm, uint32_t dt_ms);

private:
    float   _injectedX     = 0.0f;
    float   _injectedY     = 0.0f;
    float   _injectedH     = 0.0f;
    int16_t _rawX          = 0;
    int16_t _rawY          = 0;
    int16_t _rawH          = 0;
    int8_t  _linearScalar  = 0;
    int8_t  _angularScalar = 0;

    // Sim model state
    bool  _useSimModel      = false;
    float _linearNoiseSigma = 0.0f;
    float _yawNoiseSigma    = 0.0f;
    float _odomX            = 0.0f;
    float _odomY            = 0.0f;
    float _odomH            = 0.0f;

    // Sim model velocity/accel output (body frame), refreshed each tick().
    // Derived from the SAME noisy arc segment as the position model so the
    // velocity channel is consistent with the position channel.
    float _velV             = 0.0f;   // forward speed, mm/s
    float _velOmega         = 0.0f;   // yaw rate, rad/s
    float _accAx            = 0.0f;   // forward accel, mm/s^2 (finite diff of v)
    float _accAy            = 0.0f;   // lateral accel, mm/s^2 (≈0, diff drive)
    float _prevVelV         = 0.0f;

#ifdef HOST_BUILD
    std::mt19937 _rng{43u};
#endif
};
