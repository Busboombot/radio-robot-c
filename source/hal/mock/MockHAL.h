#pragma once
#include <stdint.h>
#include <cmath>
#include "../Hardware.h"
#include "MockMotor.h"
#include "MockLineSensor.h"
#include "MockColorSensor.h"
#include "MockOtosSensor.h"
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

    // Actuator-state tick (034-005): satisfies the Hardware base-class overload
    // that delivers commanded motor velocities to the HAL plant.  MockHAL's
    // plant is already driven by the single-arg tick(); this overload delegates
    // to it so the dt-guard makes it idempotent when called again from
    // loopTickOnce with the same timestamp (sim_api calls this before
    // controlCollectSplitPhase; loopTickOnce calls it after driveAdvance with
    // the same now — the dt==0 guard skips the second integration).
    void tick(uint32_t now_ms, const MotorCommands& cmds) override {
        (void)cmds;  // MockHAL plant reads motor objects directly; cmds unused
        tick(now_ms);
    }

    // Test accessors ---------------------------------------------------------
    MockMotor&       motorLMock()    { return _motorL; }
    MockMotor&       motorRMock()    { return _motorR; }
    MockLineSensor&  lineMock()      { return _line; }
    MockColorSensor& colorMock()     { return _color; }
    MockOtosSensor&  otosMock()      { return _otos; }
    MockPortIO&      portIOMock()    { return _portIO; }
    MockServo&       servoMock()     { return _servo; }

    // Exact-pose oracle (pre-slip, pre-noise ground truth).
    ExactPoseTracker& exactPoseMock() { return _exactPose; }

    // Set robot trackwidth (mm) so ExactPoseTracker can integrate correctly.
    void setTrackwidth(float mm) { _trackwidthMm = mm; }

    // Bench-OTOS swap (034-003): MockHAL tracks the toggle so that host-sim
    // tests can round-trip DBG OTOS BENCH enable/disable via the Hardware
    // interface without a NezhaHAL downcast.
    void setOtosBench(bool on) override { _benchMode = on; }
    bool isBenchMode() const   override { return _benchMode; }

private:
    MockMotor        _motorL;
    MockMotor        _motorR;
    MockLineSensor   _line;
    MockColorSensor  _color;
    MockOtosSensor   _otos;
    MockPortIO       _portIO;
    MockServo        _servo;
    uint32_t         _lastTickMs   = 0;
    ExactPoseTracker _exactPose;
    float            _trackwidthMm = 0.0f;
    bool             _benchMode    = false;  // bench-OTOS toggle (034-003)
};
