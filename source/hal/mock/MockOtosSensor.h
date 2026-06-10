#pragma once
#include <stdint.h>
#include "../IOtosSensor.h"

struct RobotConfig;

/**
 * MockOtosSensor — host-compilable IOtosSensor implementation for unit tests.
 *
 * Returns zero pose by default. Tests inject a pose via setInjectedPose().
 * All calibration/raw-position methods are no-ops returning safe defaults.
 *
 * No CODAL dependency. Compiles with plain clang++ -std=c++11 -I source.
 */
class MockOtosSensor : public IOtosSensor {
public:
    // IOtosSensor interface --------------------------------------------------
    OtosPose readTransformed(const RobotConfig& cfg) const override;

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

private:
    float   _injectedX     = 0.0f;
    float   _injectedY     = 0.0f;
    float   _injectedH     = 0.0f;
    int16_t _rawX          = 0;
    int16_t _rawY          = 0;
    int16_t _rawH          = 0;
    int8_t  _linearScalar  = 0;
    int8_t  _angularScalar = 0;
};
