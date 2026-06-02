#pragma once
#include "MicroBit.h"
#include "Config.h"

/**
 * NezhaV2 — I2C driver for the PlanetX Nezha V2 motor controller.
 *
 * I2C address: 0x10 (7-bit).
 *
 * Protocol verified against PlanetX pxt-nezha2/main.ts:
 *   Motor start (8-byte write):
 *     [0xFF, 0xF9, motorId, direction, 0x60, speed, 0xF5, 0x00]
 *     direction: 1=CW (forward), 2=CCW (reverse)
 *     speed: 0-100 (absolute)
 *
 *   Encoder read (8-byte write + 4-byte read):
 *     Write: [0xFF, 0xF9, motorId, 0x00, 0x46, 0x00, 0xF5, 0x00]
 *     Read:  4 bytes, signed int32 little-endian, units = tenths of degrees
 *
 *   Encoder zero is maintained in software (offset array), matching the
 *   TypeScript resetRelAngleValue() behaviour.
 */
class NezhaV2 { // FIXME This should be called a Motor, not a Nezha, which is the entire controller. 
public:
    explicit NezhaV2(MicroBitI2C& i2c);

    // Set raw PWM duty (-100..100). Positive = forward on both wheels.
    void    setPwm(int8_t leftPct, int8_t rightPct);

    // Read cumulative encoder in mm. leftWheel true = M2 (left), false = M1 (right).
    int32_t readEncoder(bool leftWheel, const CalibParams& cal) const;

    // Zero both encoder accumulators (software offset reset, matches chip protocol).
    void    resetEncoders();

    // FIXME also need to fill in other impoartant methods from the vendor extension, like 
    // an analog for readSpeed()

private:
    MicroBitI2C& _i2c;

    // FIXME This object should be for just one motor, not both, and it should 
    // include the PID controller for the motor, and the state information for velocity and
    // position, as well as commanded velocity. It will also need the encoder offset, like the
    // vendor code uses. 

    static constexpr uint8_t ADDR        = 0x10;
    static constexpr uint8_t LEFT_MOTOR  = 2;   // M2
    static constexpr uint8_t RIGHT_MOTOR = 1;   // M1
    static constexpr int8_t  LEFT_FWD   = +1;  // FIXME this should be a per-motor parameter, not a hardcoded constant, and should be +1 for one motor and -1 for the other, depending on how they are mounted.
    static constexpr int8_t  RIGHT_FWD  = -1;

    // Direction bytes used by the chip protocol.
    static constexpr uint8_t DIR_CW  = 1;   // positive speed
    static constexpr uint8_t DIR_CCW = 2;   // negative speed

    // Software encoder offsets (tenths of degrees), indexed by motorId-1.
    // Index 0 = M1 (RIGHT_MOTOR), index 1 = M2 (LEFT_MOTOR).
    mutable int32_t _encOffset[4];

    // Write an 8-byte motor command to the chip.
    void    writeMotorCmd(uint8_t motorId, uint8_t direction, uint8_t speed);

    // Read raw cumulative encoder from chip for one motor (tenths of degrees).
    int32_t readEncoderRaw(uint8_t motorId) const;
};
