#pragma once
#include <stdint.h>
#include "../IMotor.h"

struct RobotConfig;

/**
 * MockMotor — host-compilable IMotor implementation for unit tests.
 *
 * Integrates commanded speed into an encoder accumulator on each tick(dt_ms).
 * Physics: encoderMm += (cmdSpeed / 100.0) * kNominalMaxMms * offsetFactor * (dt_ms / 1000.0)
 *
 * No CODAL dependency. Compiles with plain clang++ -std=c++11 -I source.
 */
class MockMotor : public IMotor {
public:
    static constexpr float kNominalMaxMms = 400.0f;

    // IMotor interface -------------------------------------------------------
    void    setSpeed(int8_t pct) override;
    void    requestEncoder() override;
    int32_t collectEncoder() const override;
    float   readEncoderMmF(const RobotConfig& cfg) const override;
    float   readEncoderMmFAtomic(const RobotConfig& cfg) const override;
    float   readEncoderMmFSettle(const RobotConfig& cfg) const override;
    void    resetEncoder() override;

    // Simulation control -----------------------------------------------------

    // Advance physics by dt_ms milliseconds.
    void tick(uint32_t dt_ms);

    // Test accessors
    float  encoderMm() const { return _encoderMm; }
    int8_t cmdSpeed()  const { return _cmdSpeed; }

    // Inject a per-wheel speed offset factor (default 1.0 = symmetric).
    void setOffsetFactor(float f) { _offsetFactor = f; }

    // No-op noise stub — accepted for interface compatibility with test drivers.
    void setNoiseMms(float) {}

private:
    int8_t  _cmdSpeed     = 0;
    float   _encoderMm    = 0.0f;
    float   _offsetFactor = 1.0f;
};
