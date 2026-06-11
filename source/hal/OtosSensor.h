#pragma once
#include "MicroBit.h"
#include "I2CBus.h"
#include "IOtosSensor.h"
#include <stdint.h>

struct RobotConfig;   // fwd decl — begin() applies the OTOS scalars from config

/**
 * OtosSensor — I2C driver for the SparkFun Optical Tracking Odometry Sensor (OTOS).
 * OtosPose is defined in IOtosSensor.h (included above).
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
class OtosSensor : public IOtosSensor {
public:
    OtosSensor(I2CBus& i2c, const RobotConfig& cfg);

    // Detect (read PRODUCT_ID) and, if found: init signal processing, reset
    // Kalman tracking, and apply the linear/angular scalars from config.
    // Sets _initialized = (id == EXPECTED_PRODUCT_ID). Returns _initialized.
    bool begin() override;

    // Re-run device init: enable all signal processing (0x0F) and reset Kalman
    // tracking.  Called by begin() after detection; also exposed for the OI
    // command.  No-op if not initialized.
    void init() override;

    // Write N to REG_IMU_CALIBRATION. Calibration runs asynchronously.
    void calibrateImu(uint8_t samples) override;

    // Write 0x01 to REG_RESET (resets Kalman filters, not position).
    void resetTracking() override;

    // Read the raw position registers, convert LSBs to mm/rad, apply the
    // upside-down flip and mounting-offset rotation from cfg, and return the
    // result as an OtosPose.  Does NOT write to HardwareState or call
    // odometry.correct — those steps remain with the caller (Robot::otosCorrect).
    // Returns {0,0,0} if not initialized.
    OtosPose readTransformed(const RobotConfig& cfg) const override;

    // Read velocity registers (REG_VELOCITY_XL = 0x26), apply the same flip
    // and mounting rotation as readTransformed().  Returns {0,0} if not initialized.
    OtosVelocity readVelocityTransformed(const RobotConfig& cfg) const override;

    // Read acceleration registers (REG_ACCELERATION_XL = 0x2C), apply the same
    // flip and mounting rotation.  Returns {0,0} if not initialized.
    OtosAccel readAccelTransformed(const RobotConfig& cfg) const override;

    void getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const override;
    void setPositionRaw(int16_t x, int16_t y, int16_t h) override;
    void getVelocityRaw(int16_t& x, int16_t& y, int16_t& h) const;

    int8_t getLinearScalar() const override;
    void   setLinearScalar(int8_t val) override;
    int8_t getAngularScalar() const override;
    void   setAngularScalar(int8_t val) override;

private:
    I2CBus&            _i2c;
    const RobotConfig& _cfg;
    static constexpr uint8_t ADDR = 0x17;

    // Convert a calibration scale (e.g. 1.05) to the chip's signed-int8 scalar
    // (0.1% per LSB), clamped to [-127, 127].
    static int8_t scaleToInt8(float scale);

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
    static constexpr uint8_t REG_ACCELERATION_XL    = 0x2C;

    static constexpr uint8_t EXPECTED_PRODUCT_ID = 0x5F;

    void    writeReg8(uint8_t reg, uint8_t val);
    uint8_t readReg8(uint8_t reg) const;

    // Burst read 6 bytes from a triple-register block (X_L X_H Y_L Y_H H_L H_H).
    void readXYH(uint8_t startReg, int16_t& x, int16_t& y, int16_t& h) const;

    // Burst write three signed int16 to a triple-register block.
    void writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h);
};
