#pragma once
#include <stdint.h>
#include "IMotor.h"
#include "ILineSensor.h"
#include "IColorSensor.h"
#include "IOtosSensor.h"
#include "IPortIO.h"
#include "IServo.h"

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
};
