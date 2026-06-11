#pragma once
#include <stdint.h>
#include "Sensor.h"

struct RobotConfig;

/**
 * OtosPose — robot-frame position and heading returned by readTransformed().
 *
 * x, y: position in mm; h: heading in radians.
 * Flip and mounting-offset rotation already applied.
 */
struct OtosPose { float x, y, h; };

/**
 * OtosVelocity — body-frame velocity returned by readVelocityTransformed().
 *
 * v_mmps: forward body speed in mm/s (forward-axis projection after mounting
 *         rotation; sign preserved — positive = forward).
 * omega_rads: yaw rate in rad/s (positive = counter-clockwise).
 * Flip and mounting-offset rotation already applied.
 */
struct OtosVelocity { float v_mmps; float omega_rads; };

/**
 * OtosAccel — body-frame acceleration returned by readAccelTransformed().
 *
 * ax_mmps2: forward body acceleration in mm/s^2.
 * ay_mmps2: lateral body acceleration in mm/s^2.
 * Flip and mounting-offset rotation already applied.
 */
struct OtosAccel { float ax_mmps2; float ay_mmps2; };

/**
 * IOtosSensor — interface for the SparkFun OTOS odometry sensor.
 *
 * Extends Sensor so that begin() and is_initialized() are provided by the
 * Sensor base; concrete classes inherit from IOtosSensor only (not both
 * Sensor and IOtosSensor) to avoid diamond inheritance.
 *
 * Includes the full OtosSensor API (not just the minimal read/begin set)
 * so that Odometry command handlers (OI, OZ, OR, OP, OV, OL, OA) can
 * reach calibration and raw-position methods through the interface pointer
 * without downcasting to the concrete type.
 */
class IOtosSensor : public Sensor {
public:
    virtual ~IOtosSensor() = default;

    // Read position registers, apply transform from cfg, and return OtosPose.
    virtual OtosPose readTransformed(const RobotConfig& cfg) const = 0;

    // Read velocity registers, apply transform from cfg, and return OtosVelocity.
    // v_mmps is the forward-axis body speed; omega_rads is the yaw rate.
    virtual OtosVelocity readVelocityTransformed(const RobotConfig& cfg) const = 0;

    // Read acceleration registers, apply transform from cfg, and return OtosAccel.
    // ax_mmps2/ay_mmps2 are body-frame linear accelerations (angular discarded).
    virtual OtosAccel readAccelTransformed(const RobotConfig& cfg) const = 0;

    // Re-run device init (signal processing + Kalman reset). No-op if not inited.
    virtual void init() = 0;

    // Write N to the IMU calibration register.
    virtual void calibrateImu(uint8_t samples) = 0;

    // Reset Kalman tracking filters.
    virtual void resetTracking() = 0;

    // Raw position register access (signed int16 LSBs).
    virtual void getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const = 0;
    virtual void setPositionRaw(int16_t x, int16_t y, int16_t h) = 0;

    // Linear and angular scalar access (signed int8, 0.1% per LSB).
    virtual int8_t getLinearScalar() const = 0;
    virtual void   setLinearScalar(int8_t val) = 0;
    virtual int8_t getAngularScalar() const = 0;
    virtual void   setAngularScalar(int8_t val) = 0;
};
