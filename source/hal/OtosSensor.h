#pragma once
#include "MicroBit.h"
#include <stdint.h>

/**
 * OtosSensor — I2C driver for the SparkFun Optical Tracking Odometry Sensor (OTOS).
 *
 * I2C address: 0x17 (7-bit).
 *
 * Register map (multi-byte values are little-endian signed int16):
 *   0x00  PRODUCT_ID      (read; expected 0x5F)
 *   0x04  LINEAR_SCALAR   (signed int8, 0.1% resolution)
 *   0x05  ANGULAR_SCALAR  (signed int8, 0.1% resolution)
 *   0x06  IMU_CALIBRATION
 *   0x07  RESET           (bit 0: reset Kalman tracking)
 *   0x0E  SIGNAL_PROCESS_CFG (LUT=0x01, Accel=0x02, Rotation=0x04, Variance=0x08)
 *   0x10  OFFSET_XL       (6 bytes: X_L X_H Y_L Y_H H_L H_H)
 *   0x1F  STATUS
 *   0x20  POSITION_XL     (6 bytes, same format)
 *   0x26  VELOCITY_XL     (6 bytes)
 */
class OtosSensor {
public:
    explicit OtosSensor(MicroBitI2C& i2c);

    // Returns false if PRODUCT_ID != 0x5F (sensor not connected or wrong device).
    bool begin();

    // Enable all signal processing (0x0F) and reset Kalman tracking.
    void init();

    // Write N to REG_IMU_CALIBRATION. Calibration runs asynchronously.
    void calibrateImu(uint8_t samples);

    // Write 0x01 to REG_RESET (resets Kalman filters, not position).
    void resetTracking();

    void getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const;
    void setPositionRaw(int16_t x, int16_t y, int16_t h);
    void getVelocityRaw(int16_t& x, int16_t& y, int16_t& h) const;

    int8_t getLinearScalar() const;
    void   setLinearScalar(int8_t val);
    int8_t getAngularScalar() const;
    void   setAngularScalar(int8_t val);

private:
    MicroBitI2C& _i2c;
    static constexpr uint8_t ADDR = 0x17;

    // Register addresses
    static constexpr uint8_t REG_PRODUCT_ID        = 0x00;
    static constexpr uint8_t REG_LINEAR_SCALAR      = 0x04;
    static constexpr uint8_t REG_ANGULAR_SCALAR     = 0x05;
    static constexpr uint8_t REG_IMU_CALIBRATION    = 0x06;
    static constexpr uint8_t REG_RESET              = 0x07;
    static constexpr uint8_t REG_SIGNAL_PROCESS_CFG = 0x0E;
    static constexpr uint8_t REG_OFFSET_XL          = 0x10;
    static constexpr uint8_t REG_POSITION_XL        = 0x20;
    static constexpr uint8_t REG_VELOCITY_XL        = 0x26;

    static constexpr uint8_t EXPECTED_PRODUCT_ID = 0x5F;

    void    writeReg8(uint8_t reg, uint8_t val);
    uint8_t readReg8(uint8_t reg) const;

    // Burst read 6 bytes from a triple-register block (X_L X_H Y_L Y_H H_L H_H).
    void readXYH(uint8_t startReg, int16_t& x, int16_t& y, int16_t& h) const;

    // Burst write three signed int16 to a triple-register block.
    void writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h);
};
