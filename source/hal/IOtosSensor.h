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
