#pragma once
#include <stdint.h>
#include "IMotor.h"
#include "ILineSensor.h"
#include "IColorSensor.h"
#include "IOtosSensor.h"
#include "IPortIO.h"
#include "IServo.h"

// Forward declaration — full definition in source/control/RobotState.h.
// Sufficient here because the overload takes a const reference (034-001).
struct MotorCommands;

/**
 * Hardware — abstract HAL registry / factory base class.
 *
 * Concrete subclasses (NezhaHAL for the physical robot, MockHAL for host
 * unit tests) own all device objects and expose them through this interface.
 * Robot takes a Hardware& in its constructor and binds interface refs from
 * the accessors below.
 */
class Hardware {
public:
    virtual ~Hardware() = default;

    virtual IMotor&       motorL()      = 0;
    virtual IMotor&       motorR()      = 0;
    virtual ILineSensor&  lineSensor()  = 0;
    virtual IColorSensor& colorSensor() = 0;
    virtual IOtosSensor&  otos()        = 0;
    virtual IPortIO&      portIO()      = 0;
    virtual IServo&       gripper()     = 0;

    // Initialize all owned devices (calls begin() on sensors).
    virtual void begin() = 0;

    // Periodic tick, called once per cooperative loop iteration.
    virtual void tick(uint32_t now_ms) = 0;

    // Actuator-state tick — delivers commanded motor velocities to the HAL
    // so that bench-mode sensor plants (BenchOtosSensor) can integrate them
    // without requiring a downcast in Robot (034-001).
    //
    // Default no-op: subclasses that do not use bench mode (MockHAL until
    // ticket 005, any future minimal HAL) inherit this and require no change.
    // NezhaHAL overrides this to drive BenchOtosSensor when bench mode is on.
    virtual void tick(uint32_t now_ms, const MotorCommands& cmds) {
        (void)now_ms;
        (void)cmds;
    }

    // Bench-OTOS swap (034-003): redirect otos() to the bench sensor (on=true)
    // or restore the real sensor (on=false).  Default no-op for HALs that do
    // not support bench mode (MockHAL uses the overrides below; NezhaHAL
    // overrides with the real swap).
    virtual void setOtosBench(bool on) { (void)on; }

    // Returns true when the bench OTOS sensor is currently active (034-003).
    // Default false for HALs that do not support bench mode.
    virtual bool isBenchMode() const { return false; }
};
