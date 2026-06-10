#pragma once
#include <stdint.h>

struct RobotConfig;

/**
 * IMotor — pure-virtual interface for a single drive motor.
 *
 * Allows MotorController and Robot to be written against the interface
 * rather than the concrete Motor class, enabling test doubles and future
 * alternative hardware back-ends.
 */
class IMotor {
public:
    virtual ~IMotor() = default;

    // Set speed as signed percentage (-100..100). Positive = logical forward.
    virtual void setSpeed(int8_t pct) = 0;

    // Split-phase encoder I/O, phase 1: issue the 0x46 write and return.
    virtual void requestEncoder() = 0;

    // Split-phase encoder I/O, phase 2: read back the 4-byte response.
    virtual int32_t collectEncoder() const = 0;

    // High-resolution encoder read in mm as float (used by velocity loop).
    // readEncoderMmF is the generic name; concrete variants below for
    // specific timing contexts.
    virtual float readEncoderMmF(const RobotConfig& cfg) const = 0;

    // Atomic single-shot encoder read (~8 ms, safe outside control loop).
    virtual float readEncoderMmFAtomic(const RobotConfig& cfg) const = 0;

    // Settle-only encoder read (~4 ms, safe inside fixed-rate control loop).
    virtual float readEncoderMmFSettle(const RobotConfig& cfg) const = 0;

    // Zero this motor's encoder accumulator (software offset reset).
    virtual void resetEncoder() = 0;
};
