#include "OtosSensor.h"

OtosSensor::OtosSensor(MicroBitI2C& i2c)
    : _i2c(i2c)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

bool OtosSensor::begin()
{
    uint8_t id = readReg8(REG_PRODUCT_ID);
    return id == EXPECTED_PRODUCT_ID;
}

void OtosSensor::init()
{
    // Enable all signal processing: LUT | Accel | Rotation | Variance = 0x0F.
    writeReg8(REG_SIGNAL_PROCESS_CFG, 0x0F);
    // Reset Kalman tracking (bit 0 = 1).
    writeReg8(REG_RESET, 0x01);
}

void OtosSensor::calibrateImu(uint8_t samples)
{
    writeReg8(REG_IMU_CALIBRATION, samples);
}

void OtosSensor::resetTracking()
{
    writeReg8(REG_RESET, 0x01);
}

void OtosSensor::getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const
{
    readXYH(REG_POSITION_XL, x, y, h);
}

void OtosSensor::setPositionRaw(int16_t x, int16_t y, int16_t h)
{
    writeXYH(REG_POSITION_XL, x, y, h);
}

void OtosSensor::getVelocityRaw(int16_t& x, int16_t& y, int16_t& h) const
{
    readXYH(REG_VELOCITY_XL, x, y, h);
}

int8_t OtosSensor::getLinearScalar() const
{
    return (int8_t)readReg8(REG_LINEAR_SCALAR);
}

void OtosSensor::setLinearScalar(int8_t val)
{
    writeReg8(REG_LINEAR_SCALAR, (uint8_t)val);
}

int8_t OtosSensor::getAngularScalar() const
{
    return (int8_t)readReg8(REG_ANGULAR_SCALAR);
}

void OtosSensor::setAngularScalar(int8_t val)
{
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
    _i2c.write((ADDR << 1), (uint8_t*)&reg, 1, false);
    _i2c.read((ADDR << 1), (uint8_t*)&result, 1, false);
    return result;
}

void OtosSensor::readXYH(uint8_t startReg, int16_t& x, int16_t& y, int16_t& h) const
{
    uint8_t raw[6] = {0};
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
