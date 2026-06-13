#pragma once
#include <stdint.h>
#include <cmath>
#include "../Hardware.h"
#include "MockMotor.h"
#include "MockLineSensor.h"
#include "MockColorSensor.h"
#include "MockOtosSensor.h"
#include "../BenchOtosSensor.h"
#include "MockPortIO.h"
#include "MockServo.h"

/**
 * ExactPoseTracker — oracle ground-truth pose integrator.
 *
 * Uses midpoint integration identical to Odometry::predict but reads
 * trueVelocityMms() (pre-slip) from each motor, so it is unaffected by
 * the slip and noise model. Useful as a reference pose in tests.
 */
struct ExactPoseTracker {
    float x = 0.0f, y = 0.0f, h = 0.0f;

    void reset() { x = y = h = 0.0f; }

    void update(float velLMms, float velRMms, float trackwidthMm, uint32_t dt_ms) {
        float dt_s = dt_ms / 1000.0f;
        float dL   = velLMms * dt_s;
        float dR   = velRMms * dt_s;
        float dC   = (dL + dR) * 0.5f;
        float dTh  = (dR - dL) / trackwidthMm;
        float hMid = h + dTh * 0.5f;
        x += dC * cosf(hMid);
        y += dC * sinf(hMid);
        h += dTh;
    }
};

/**
 * MockHAL — host-compilable Hardware implementation for unit tests.
 *
 * Owns all six mock devices as value members. tick(now_ms) computes a signed
 * dt_ms and advances each device that has a tick() method.
 *
 * No CODAL dependency. Compiles with plain clang++ -std=c++11 -I source.
 *
 * Bench OTOS (mirrors NezhaHAL): MockHAL also owns the real firmware
 * BenchOtosSensor and an active-OTOS pointer.  setOtosBench(true) redirects
 * otos() to the bench sensor — the SAME synthetic-pose device the firmware
 * runs on hardware — so a sim run with bench mode on integrates the identical
 * BenchOtosSensor the bench does.  tick(now,cmds) feeds it the commanded wheel
 * velocity each step (the cmds the firmware loop already passes through
 * Hardware::tick).
 *
 * Test code accesses the underlying mock objects via motorLMock(),
 * motorRMock(), and otosMock() to inject state or inspect results.
 */
class MockHAL : public Hardware {
public:
    // The host sim does not call hal.begin() (unlike main.cpp on hardware), so
    // initialize the bench sensor here — otherwise BenchOtosSensor::tick() is a
    // no-op (the !_initialized guard) and bench mode would silently do nothing.
    MockHAL() { _benchOtos.begin(); }

    // Hardware interface -----------------------------------------------------
    IMotor&       motorL()      override { return _motorL; }
    IMotor&       motorR()      override { return _motorR; }
    ILineSensor&  lineSensor()  override { return _line; }
    IColorSensor& colorSensor() override { return _color; }
    // Active OTOS — real mock sensor, or the bench sensor when bench mode is on.
    IOtosSensor&  otos()        override { return *_otosActive; }
    IPortIO&      portIO()      override { return _portIO; }
    IServo&       gripper()     override { return _servo; }

    void begin() override { _benchOtos.begin(); }
    void tick(uint32_t now_ms) override;
    void tick(uint32_t now_ms, const MotorCommands& cmds) override;

    // Test accessors ---------------------------------------------------------
    MockMotor&       motorLMock()    { return _motorL; }
    MockMotor&       motorRMock()    { return _motorR; }
    MockLineSensor&  lineMock()      { return _line; }
    MockColorSensor& colorMock()     { return _color; }
    MockOtosSensor&  otosMock()      { return _otos; }
    MockPortIO&      portIOMock()    { return _portIO; }
    MockServo&       servoMock()     { return _servo; }
    BenchOtosSensor* benchOtosPtr()  { return &_benchOtos; }

    // Exact-pose oracle (pre-slip, pre-noise ground truth).
    ExactPoseTracker& exactPoseMock() { return _exactPose; }

    // Set robot trackwidth (mm) so ExactPoseTracker / the bench sensor integrate
    // correctly.
    void setTrackwidth(float mm) { _trackwidthMm = mm; }

    // Bench-OTOS swap (mirrors NezhaHAL): redirect the active OTOS pointer to
    // the bench sensor (on=true) or the real mock sensor (on=false).  The sim
    // then runs the SAME BenchOtosSensor the firmware runs on hardware.
    void setOtosBench(bool on) override {
        _otosActive = on
            ? static_cast<IOtosSensor*>(&_benchOtos)
            : static_cast<IOtosSensor*>(&_otos);
    }
    bool isBenchMode() const override {
        return _otosActive == static_cast<const IOtosSensor*>(&_benchOtos);
    }

private:
    // Shared dt-guarded plant integration for both tick() overloads.
    // cmds is non-null only on the actuator-state tick; when bench mode is
    // active it feeds the BenchOtosSensor the commanded wheel velocity.
    void advance(uint32_t now_ms, const MotorCommands* cmds);

    MockMotor        _motorL;
    MockMotor        _motorR;
    MockLineSensor   _line;
    MockColorSensor  _color;
    MockOtosSensor   _otos;
    BenchOtosSensor  _benchOtos;
    MockPortIO       _portIO;
    MockServo        _servo;
    uint32_t         _lastTickMs   = 0;
    ExactPoseTracker _exactPose;
    float            _trackwidthMm = 0.0f;
    // Active OTOS pointer — _otos (real) by default; _benchOtos when bench mode.
    IOtosSensor*     _otosActive   = &_otos;
};
