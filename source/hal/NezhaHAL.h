#pragma once
#include "MicroBit.h"
#include "Hardware.h"
#include "I2CBus.h"
#include "Motor.h"
#include "OtosSensor.h"
#include "BenchOtosSensor.h"
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
 *
 * Active-OTOS pointer (sprint 031):
 *   _otosActive initially points to _otos (real sensor).
 *   setOtosBench(true) redirects it to _benchOtos so that otos() returns
 *   the bench sensor — transparent to Robot and the rest of the firmware.
 *   setOtosBench(false) restores the real sensor.
 */
class NezhaHAL : public Hardware {
public:
    NezhaHAL(MicroBitI2C& i2c, MicroBitIO& io, const RobotConfig& cfg);

    // Hardware interface overrides ----------------------------------------
    IMotor&       motorL()      override { return _motorL; }
    IMotor&       motorR()      override { return _motorR; }
    ILineSensor&  lineSensor()  override { return _line; }
    IColorSensor& colorSensor() override { return _color; }
    // otos() returns the ACTIVE sensor — real or bench depending on _otosActive.
    IOtosSensor&  otos()        override { return *_otosActive; }
    IPortIO&      portIO()      override { return _portio; }
    IServo&       gripper()     override { return _gripper; }

    // Call otos.begin(), line.begin(), color.begin(); also begin() bench sensor.
    void begin() override;

    // No-op: devices are self-contained.
    void tick(uint32_t now_ms) override { (void)now_ms; }

    // Actuator-state tick (034-001): integrates commanded velocities into the
    // bench OTOS plant when bench mode is active; no-op when bench mode is off.
    // Ports the signed-delta dt logic from Robot::benchOtosTick so Robot no
    // longer needs a downcast.
    void tick(uint32_t now_ms, const MotorCommands& cmds) override;

    // Expose the shared I2CBus for MotorController::setI2CBus().
    I2CBus& bus() { return _bus; }

    // --- Bench OTOS swap (sprint 031) ---

    // Redirect the active OTOS pointer to the bench sensor (on=true) or
    // restore the real sensor (on=false).  Overrides Hardware::setOtosBench
    // (034-003).
    void setOtosBench(bool on) override {
        _otosActive = on
            ? static_cast<IOtosSensor*>(&_benchOtos)
            : static_cast<IOtosSensor*>(&_otos);
    }

    // Direct accessor to the BenchOtosSensor for tick() calls and noise tuning.
    BenchOtosSensor* benchOtosPtr() { return &_benchOtos; }

    // Returns true when the bench sensor is currently active.  Overrides
    // Hardware::isBenchMode (034-003).
    bool isBenchMode() const override {
        return _otosActive == static_cast<const IOtosSensor*>(&_benchOtos);
    }

private:
    I2CBus           _bus;
    Motor            _motorL;
    Motor            _motorR;
    OtosSensor       _otos;
    BenchOtosSensor  _benchOtos;
    LineSensor       _line;
    ColorSensor      _color;
    PortIO           _portio;
    Servo            _gripper;

    // Active OTOS pointer — initialized to &_otos in the constructor.
    // Must be declared AFTER both _otos and _benchOtos so those members
    // are fully constructed before _otosActive is assigned.
    IOtosSensor*     _otosActive;

    // Bench-tick state (034-001): trackwidth cached from RobotConfig at
    // construction; last-tick timestamp for signed-delta dt computation
    // (mirrors the logic formerly in Robot::benchOtosTick).
    float            _trackwidthMm    = 0.0f;
    uint32_t         _lastBenchTickMs = 0u;
};
