#include "OtosSensor.h"
#include "Config.h"
#include <cmath>

OtosSensor::OtosSensor(I2CBus& i2c, const RobotConfig& cfg)
    : _i2c(i2c), _cfg(cfg)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

int8_t OtosSensor::scaleToInt8(float scale)
{
    // scalar = clamp(round((scale - 1.0) / 0.001), -127, 127).
    // E.g. 1.05 → +50; 0.987 → -13.
    float raw = roundf((scale - 1.0f) / 0.001f);
    if (raw >  127.0f) raw =  127.0f;
    if (raw < -127.0f) raw = -127.0f;
    return static_cast<int8_t>(raw);
}

bool OtosSensor::begin()
{
    // Detect via product-ID read.
    uint8_t id = readReg8(REG_PRODUCT_ID);
    _initialized = (id == EXPECTED_PRODUCT_ID);
    if (!_initialized) return false;

    // Enable signal processing + reset Kalman tracking, then apply the
    // linear/angular calibration scalars from config.
    init();
    setLinearScalar(scaleToInt8(_cfg.otosLinearScale));
    setAngularScalar(scaleToInt8(_cfg.otosAngularScale));
    return true;
}

void OtosSensor::init()
{
    if (!is_initialized()) return;
    // Enable all signal processing: LUT | Accel | Rotation | Variance = 0x0F.
    writeReg8(REG_SIGNAL_PROCESS_CFG, 0x0F);
    // Reset Kalman tracking (bit 0 = 1).
    writeReg8(REG_RESET, 0x01);
}

void OtosSensor::calibrateImu(uint8_t samples)
{
    if (!is_initialized()) return;
    writeReg8(REG_IMU_CALIBRATION, samples);
}

void OtosSensor::resetTracking()
{
    if (!is_initialized()) return;
    writeReg8(REG_RESET, 0x01);
}

OtosPose OtosSensor::readTransformed(const RobotConfig& cfg) const
{
    if (!is_initialized()) return {0.0f, 0.0f, 0.0f};

    int16_t rx = 0, ry = 0, rh = 0;
    readXYH(REG_POSITION_XL, rx, ry, rh);

    constexpr float kPosMmPerLsb  = 0.305f;
    constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);

    float xF = static_cast<float>(rx) * kPosMmPerLsb;
    float yF = static_cast<float>(ry) * kPosMmPerLsb;
    float hF = static_cast<float>(rh) * kHdgRadPerLsb;

    if (cfg.odomUpsideDown) {
        xF = -xF;
        yF = -yF;
        hF = -hF;
    }

    float angRad = -cfg.odomYawDeg * (3.14159265f / 180.0f);
    float c = cosf(angRad);
    float s = sinf(angRad);

    OtosPose pose;
    pose.x = c * xF - s * yF - cfg.odomOffX;
    pose.y = s * xF + c * yF - cfg.odomOffY;
    pose.h = hF + cfg.odomYawDeg * (3.14159265f / 180.0f);
    return pose;
}

// ---------------------------------------------------------------------------
// OTOS LSB Scale Factors (Sprint 023)
//
// The SparkFun OTOS chip uses the same signed-int16 register layout for
// position, velocity, and acceleration.  All three register banks share
// identical LSB resolutions:
//
//   Linear:  1 LSB = 1/32768 m  =  ~0.030518 mm  ≈ 0.305 mm / 10
//            BUT the chip operates on a 10-LSB-per-unit scale internally,
//            so effective linear scale = 0.305 mm/LSB  (= 1/32768 m * 10).
//   Angular: 1 LSB = 0.00549 deg  (= 1/32768 rev * 65.536, ~360/65536 deg)
//
// Sources:
//   • SparkFun OTOS Arduino library (SparkFun_Optical_Tracking_Odometry_Sensor.h):
//       kMeterToInt16 = 32768/meter → 1 LSB = 1/32768 m ≈ 0.030518 mm
//       The library further scales by INT_TO_FLOAT_METERS = 1/kMeterToInt16
//       and the internal factor of 10 used by readPosition/readVelocity, so
//       effective per-register scale = 0.305 mm/LSB (matches kPosMmPerLsb).
//   • The velocity registers (0x26) and acceleration registers (0x2C) use the
//     SAME bit-for-bit LSB encoding as the position registers (0x20), so
//     kVelMmpsPerLsb = kPosMmPerLsb = 0.305 mm/s/LSB and
//     kAccMmps2PerLsb = kPosMmPerLsb = 0.305 mm/s²/LSB.
//   • Angular rate uses kHdgDegPerLsb = 0.00549 deg/LSB → same value for
//     omega (deg/s per LSB); converted to rad/s below.
//
// Body-frame v derivation:
//   After applying the mounting rotation (odomYawDeg flip+rotation), the
//   OTOS x-axis points forward in the robot body frame.  Body speed is
//   taken as vx_body (the forward-axis component) rather than the vector
//   magnitude |v|.  For a differential-drive robot vy should be near zero;
//   using vx avoids sign ambiguity from sqrtf and correctly handles both
//   forward and reverse motion.  omega_rads comes directly from the heading
//   channel of the velocity register (deg/s → rad/s), sign-preserved.
// ---------------------------------------------------------------------------

OtosVelocity OtosSensor::readVelocityTransformed(const RobotConfig& cfg) const
{
    if (!is_initialized()) return {0.0f, 0.0f};

    int16_t rvx = 0, rvy = 0, rvh = 0;
    readXYH(REG_VELOCITY_XL, rvx, rvy, rvh);

    // Same LSB resolution as position (see comment block above).
    constexpr float kVelMmpsPerLsb   = 0.305f;           // mm/s per LSB
    constexpr float kOmegaRadpsPerLsb = 0.00549f * (3.14159265f / 180.0f); // rad/s per LSB

    float vxF = static_cast<float>(rvx) * kVelMmpsPerLsb;
    float vyF = static_cast<float>(rvy) * kVelMmpsPerLsb;
    float whF = static_cast<float>(rvh) * kOmegaRadpsPerLsb;

    if (cfg.odomUpsideDown) {
        vxF = -vxF;
        vyF = -vyF;
        whF = -whF;
    }

    float angRad = -cfg.odomYawDeg * (3.14159265f / 180.0f);
    float c = cosf(angRad);
    float s = sinf(angRad);

    // Rotate into body frame; vx_body is the forward-axis projection.
    // odomYawDeg is a constant mounting offset to heading; its derivative is
    // zero, so omega passes through unchanged (after the flip above).
    float vxBody = c * vxF - s * vyF;

    OtosVelocity vel;
    vel.v_mmps     = vxBody;
    vel.omega_rads = whF;
    return vel;
}

OtosAccel OtosSensor::readAccelTransformed(const RobotConfig& cfg) const
{
    if (!is_initialized()) return {0.0f, 0.0f};

    int16_t rax = 0, ray = 0, rah = 0;
    readXYH(REG_ACCELERATION_XL, rax, ray, rah);

    // Same LSB resolution as position (see comment block above).
    constexpr float kAccMmps2PerLsb = 0.305f;  // mm/s² per LSB

    float axF = static_cast<float>(rax) * kAccMmps2PerLsb;
    float ayF = static_cast<float>(ray) * kAccMmps2PerLsb;
    // rah (angular acceleration) is discarded — only linear acceleration is used.
    (void)rah;

    if (cfg.odomUpsideDown) {
        axF = -axF;
        ayF = -ayF;
    }

    float angRad = -cfg.odomYawDeg * (3.14159265f / 180.0f);
    float c = cosf(angRad);
    float s = sinf(angRad);

    OtosAccel accel;
    accel.ax_mmps2 = c * axF - s * ayF;
    accel.ay_mmps2 = s * axF + c * ayF;
    return accel;
}

void OtosSensor::getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const
{
    if (!is_initialized()) { x = 0; y = 0; h = 0; return; }
    readXYH(REG_POSITION_XL, x, y, h);
}

void OtosSensor::setPositionRaw(int16_t x, int16_t y, int16_t h)
{
    if (!is_initialized()) return;
    writeXYH(REG_POSITION_XL, x, y, h);
}

void OtosSensor::getVelocityRaw(int16_t& x, int16_t& y, int16_t& h) const
{
    if (!is_initialized()) { x = 0; y = 0; h = 0; return; }
    readXYH(REG_VELOCITY_XL, x, y, h);
}

int8_t OtosSensor::getLinearScalar() const
{
    if (!is_initialized()) return 0;
    return (int8_t)readReg8(REG_LINEAR_SCALAR);
}

void OtosSensor::setLinearScalar(int8_t val)
{
    if (!is_initialized()) return;
    writeReg8(REG_LINEAR_SCALAR, (uint8_t)val);
}

int8_t OtosSensor::getAngularScalar() const
{
    if (!is_initialized()) return 0;
    return (int8_t)readReg8(REG_ANGULAR_SCALAR);
}

void OtosSensor::setAngularScalar(int8_t val)
{
    if (!is_initialized()) return;
    writeReg8(REG_ANGULAR_SCALAR, (uint8_t)val);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void OtosSensor::writeReg8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    _i2c.write((ADDR << 1), (uint8_t*)buf, 2, false);
}

uint8_t OtosSensor::readReg8(uint8_t reg) const
{
    uint8_t result = 0;
    // repeated=true → no STOP after the register write (repeated-start read).
    _i2c.write((ADDR << 1), (uint8_t*)&reg, 1, false);
    _i2c.read((ADDR << 1), (uint8_t*)&result, 1, false);
    return result;
}

void OtosSensor::readXYH(uint8_t startReg, int16_t& x, int16_t& y, int16_t& h) const
{
    uint8_t raw[6] = {0};
    // repeated=true → no STOP after the register write (repeated-start read).
    _i2c.write((ADDR << 1), (uint8_t*)&startReg, 1, false);
    _i2c.read((ADDR << 1), (uint8_t*)raw, 6, false);
    x = (int16_t)(raw[0] | ((uint16_t)raw[1] << 8));
    y = (int16_t)(raw[2] | ((uint16_t)raw[3] << 8));
    h = (int16_t)(raw[4] | ((uint16_t)raw[5] << 8));
}

void OtosSensor::writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h)
{
    uint8_t buf[7];
    buf[0] = startReg;
    buf[1] = (uint8_t)(x & 0xFF);
    buf[2] = (uint8_t)((x >> 8) & 0xFF);
    buf[3] = (uint8_t)(y & 0xFF);
    buf[4] = (uint8_t)((y >> 8) & 0xFF);
    buf[5] = (uint8_t)(h & 0xFF);
    buf[6] = (uint8_t)((h >> 8) & 0xFF);
    _i2c.write((ADDR << 1), (uint8_t*)buf, 7, false);
}
