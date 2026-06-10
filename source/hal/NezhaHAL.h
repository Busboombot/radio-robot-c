#pragma once
#include "MicroBit.h"
#include "Hardware.h"
#include "I2CBus.h"
#include "Motor.h"
#include "OtosSensor.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "PortIO.h"
#include "Servo.h"
#include "Config.h"

/**
 * NezhaHAL — concrete HAL implementation for the PlanetX Nezha V2 robot.
 *
 * Owns all seven device objects as value members (no heap allocation).
 * Constructed in main() as a static and passed to Robot as Hardware&.
 *
 * begin() initialises the sensors in the order: OTOS, line, color.
 * tick() is a no-op (devices are self-contained / interrupt-driven).
 *
 * bus() exposes the shared I2CBus so main.cpp can wire it into
 * MotorController::setI2CBus() for the enc_wedge diagnostic.
 *
 * NOTE: This header includes CODAL/MicroBit headers and must NOT be
 * included from host-build translation units. The host build (ticket 003)
 * will use MockHAL instead.
 */
class NezhaHAL : public Hardware {
public:
    NezhaHAL(MicroBitI2C& i2c, MicroBitIO& io, const RobotConfig& cfg);

    // Hardware interface overrides ----------------------------------------
    IMotor&       motorL()      override { return _motorL; }
    IMotor&       motorR()      override { return _motorR; }
    ILineSensor&  lineSensor()  override { return _line; }
    IColorSensor& colorSensor() override { return _color; }
    IOtosSensor&  otos()        override { return _otos; }
    IPortIO&      portIO()      override { return _portio; }
    IServo&       gripper()     override { return _gripper; }

    // Call otos.begin(), line.begin(), color.begin().
    void begin() override;

    // No-op: devices are self-contained.
    void tick(uint32_t now_ms) override { (void)now_ms; }

    // Expose the shared I2CBus for MotorController::setI2CBus().
    I2CBus& bus() { return _bus; }

private:
    I2CBus      _bus;
    Motor       _motorL;
    Motor       _motorR;
    OtosSensor  _otos;
    LineSensor  _line;
    ColorSensor _color;
    PortIO      _portio;
    Servo       _gripper;
};
