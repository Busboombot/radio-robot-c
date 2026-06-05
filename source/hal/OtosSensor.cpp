#include "OtosSensor.h"
#include "Config.h"
#include <cmath>

OtosSensor::OtosSensor(MicroBitI2C& i2c, const RobotConfig& cfg)
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
    // linear/angular calibration scalars from config.  (init() and the
    // setter calls are gated on is_initialized(), now true.)
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
