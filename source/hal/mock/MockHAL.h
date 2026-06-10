#pragma once
#include <stdint.h>
#include "../Hardware.h"
#include "MockMotor.h"
#include "MockLineSensor.h"
#include "MockColorSensor.h"
#include "MockOtosSensor.h"
#include "MockPortIO.h"
#include "MockServo.h"

/**
 * MockHAL — host-compilable Hardware implementation for unit tests.
 *
 * Owns all six mock devices as value members. tick(now_ms) computes a signed
 * dt_ms and advances each device that has a tick() method.
 *
 * No CODAL dependency. Compiles with plain clang++ -std=c++11 -I source.
 *
 * Test code accesses the underlying mock objects via motorLMock(),
 * motorRMock(), and otosMock() to inject state or inspect results.
 */
class MockHAL : public Hardware {
public:
    // Hardware interface -----------------------------------------------------
    IMotor&       motorL()      override { return _motorL; }
    IMotor&       motorR()      override { return _motorR; }
    ILineSensor&  lineSensor()  override { return _line; }
    IColorSensor& colorSensor() override { return _color; }
    IOtosSensor&  otos()        override { return _otos; }
    IPortIO&      portIO()      override { return _portIO; }
    IServo&       gripper()     override { return _servo; }

    void begin() override {}
    void tick(uint32_t now_ms) override;

    // Test accessors ---------------------------------------------------------
    MockMotor&      motorLMock()  { return _motorL; }
    MockMotor&      motorRMock()  { return _motorR; }
    MockLineSensor& lineMock()    { return _line; }
    MockColorSensor& colorMock()  { return _color; }
    MockOtosSensor& otosMock()    { return _otos; }
    MockPortIO&     portIOMock()  { return _portIO; }
    MockServo&      servoMock()   { return _servo; }

private:
    MockMotor       _motorL;
    MockMotor       _motorR;
    MockLineSensor  _line;
    MockColorSensor _color;
    MockOtosSensor  _otos;
    MockPortIO      _portIO;
    MockServo       _servo;
    uint32_t        _lastTickMs = 0;
};
