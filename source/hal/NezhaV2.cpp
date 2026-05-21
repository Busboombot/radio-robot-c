#include "hal/NezhaV2.h"

// ---------------------------------------------------------------------------
// I2C wire protocol constants (verified against PlanetX pxt-nezha2/main.ts)
// ---------------------------------------------------------------------------
//
// Every command is an 8-byte write to address 0x10.
// The frame always starts with 0xFF 0xF9 followed by motor-id, then
// a command-specific payload in bytes [3..7].
//
// Motor start (__start):
//   [0xFF, 0xF9, motorId, direction, 0x60, speed, 0xF5, 0x00]
//   direction: 1 = CW (positive speed), 2 = CCW (negative speed)
//   speed: absolute value 0-100
//
// Motor stop (stop):
//   [0xFF, 0xF9, motorId, 0x00, 0x5F, 0x00, 0xF5, 0x00]
//
// Encoder read (readAngle):
//   Write: [0xFF, 0xF9, motorId, 0x00, 0x46, 0x00, 0xF5, 0x00]
//   Read:  4 bytes, signed int32 little-endian, units = tenths of degrees
// ---------------------------------------------------------------------------

NezhaV2::NezhaV2(MicroBitI2C& i2c)
    : _i2c(i2c)
{
    _encOffset[0] = 0;
    _encOffset[1] = 0;
    _encOffset[2] = 0;
    _encOffset[3] = 0;
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void NezhaV2::setPwm(int8_t leftPct, int8_t rightPct)
{
    // Clamp to [-100, 100].
    if (leftPct  >  100) leftPct  =  100;
    if (leftPct  < -100) leftPct  = -100;
    if (rightPct >  100) rightPct =  100;
    if (rightPct < -100) rightPct = -100;

    // Left wheel is M2 (LEFT_MOTOR).  Positive leftPct = forward = CW.
    // Apply LEFT_FWD (+1) sign — no inversion needed.
    if (leftPct == 0) {
        // Zero speed: send the explicit stop command.
        uint8_t stopBuf[8] = {0xFF, 0xF9, LEFT_MOTOR, 0x00, 0x5F, 0x00, 0xF5, 0x00};
        _i2c.write((ADDR << 1), (const char*)stopBuf, 8, false);
    } else {
        uint8_t dir   = (leftPct > 0) ? DIR_CW : DIR_CCW;
        uint8_t speed = (leftPct > 0) ? (uint8_t)leftPct : (uint8_t)(-leftPct);
        writeMotorCmd(LEFT_MOTOR, dir, speed);
    }

    // Right wheel is M1 (RIGHT_MOTOR).  RIGHT_FWD = -1 means positive rightPct
    // argument means "forward" but the physical motor must spin CCW to go forward.
    int8_t rightEffective = (int8_t)(RIGHT_FWD * rightPct);  // flip the sign
    if (rightEffective == 0) {
        uint8_t stopBuf[8] = {0xFF, 0xF9, RIGHT_MOTOR, 0x00, 0x5F, 0x00, 0xF5, 0x00};
        _i2c.write((ADDR << 1), (const char*)stopBuf, 8, false);
    } else {
        uint8_t dir   = (rightEffective > 0) ? DIR_CW : DIR_CCW;
        uint8_t speed = (rightEffective > 0) ? (uint8_t)rightEffective
                                              : (uint8_t)(-rightEffective);
        writeMotorCmd(RIGHT_MOTOR, dir, speed);
    }
}

int32_t NezhaV2::readEncoder(bool leftWheel, const CalibParams& cal) const
{
    uint8_t motorId = leftWheel ? LEFT_MOTOR : RIGHT_MOTOR;
    int32_t raw     = readEncoderRaw(motorId);   // tenths of degrees

    // Mirror TypeScript: (raw / 10.0) * mmPerDeg * fwdSign
    float degF = raw / 10.0f;
    float mmF  = degF
                 * (leftWheel ? cal.mmPerDegL : cal.mmPerDegR)
                 * (leftWheel ? (float)LEFT_FWD : (float)RIGHT_FWD);
    return (int32_t)mmF;
}

void NezhaV2::resetEncoders()
{
    // Mirror TypeScript resetRelAngleValue(): snapshot the current raw
    // angle into the software offset so that subsequent reads return zero.
    _encOffset[LEFT_MOTOR  - 1] = readEncoderRaw(LEFT_MOTOR);
    _encOffset[RIGHT_MOTOR - 1] = readEncoderRaw(RIGHT_MOTOR);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void NezhaV2::writeMotorCmd(uint8_t motorId, uint8_t direction, uint8_t speed)
{
    uint8_t buf[8] = {
        0xFF,
        0xF9,
        motorId,
        direction,
        0x60,
        speed,
        0xF5,
        0x00
    };
    _i2c.write((ADDR << 1), (const char*)buf, 8, false);
}

int32_t NezhaV2::readEncoderRaw(uint8_t motorId) const
{
    // Send encoder-read command.
    uint8_t cmd[8] = {
        0xFF, 0xF9,
        motorId,
        0x00, 0x46,
        0x00, 0xF5,
        0x00
    };
    _i2c.write((ADDR << 1), (const char*)cmd, 8, false);

    // Read 4 bytes (signed int32, little-endian).
    uint8_t resp[4] = {0, 0, 0, 0};
    _i2c.read((ADDR << 1), (char*)resp, 4, false);

    int32_t raw = (int32_t)(
        ((uint32_t)resp[3] << 24) |
        ((uint32_t)resp[2] << 16) |
        ((uint32_t)resp[1] <<  8) |
        ((uint32_t)resp[0])
    );

    // Subtract the software offset captured at last resetEncoders() call.
    return raw - _encOffset[motorId - 1];
}
