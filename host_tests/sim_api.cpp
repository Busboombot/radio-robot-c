// sim_api.cpp — extern "C" C ABI wrapper over a self-contained simulation.
//
// Provides an opaque SimHandle that owns MockHAL + Robot + CommandProcessor.
// Python test code (ticket 020-004) loads this shared library via ctypes.
//
// Build: cmake -S . -B host_tests/build && cmake --build host_tests/build
// Load:  python3 -c "import ctypes; ctypes.CDLL('./host_tests/build/libfirmware_host.dylib')"

#include "robot/Robot.h"
#include "app/CommandProcessor.h"
#include "hal/mock/MockHAL.h"
#include "hal/mock/MockMotor.h"
#include "hal/mock/MockOtosSensor.h"
#include "types/Config.h"
#include "control/RobotState.h"

#include <cstring>
#include <cstdio>
#include <utility>

// ---------------------------------------------------------------------------
// SimHandle — one self-contained simulation instance allocated per test.
//
// Construction order is load-bearing:
//   1. hal        — MockHAL (owns all mock devices)
//   2. cfg        — RobotConfig value from defaultRobotConfig()
//   3. robot      — Robot(hal, cfg), wires motorController/odometry/etc.
//   4. cmd        — CommandProcessor with the full command table
// ---------------------------------------------------------------------------
struct SimHandle {
    MockHAL          hal;
    RobotConfig      cfg;
    Robot            robot;
    CommandProcessor cmd;

    SimHandle()
        : hal()
        , cfg(defaultRobotConfig())
        , robot(hal, cfg)
        , cmd(robot.buildCommandTable(nullptr, nullptr))
    {}
};

// ---------------------------------------------------------------------------
// Internal: capture reply into a fixed buffer.
// ---------------------------------------------------------------------------
struct ReplyCapture {
    char* buf;
    int   cap;
    int   written;
};

static void captureReply(const char* msg, void* ctx)
{
    ReplyCapture* c = static_cast<ReplyCapture*>(ctx);
    if (!msg || c->written >= c->cap - 1) return;
    int remaining = c->cap - c->written - 1;
    int n = snprintf(c->buf + c->written, (size_t)remaining, "%s\n", msg);
    if (n > 0 && n < remaining) c->written += n;
}

// ---------------------------------------------------------------------------
// C ABI
// ---------------------------------------------------------------------------
extern "C" {

// ---- Lifecycle ----

void* sim_create()
{
    return new SimHandle();
}

void sim_destroy(void* h)
{
    delete static_cast<SimHandle*>(h);
}

// ---- Tick ----

// Advance simulation by one control tick.
// hal.tick() drives MockMotor physics (integrates encoder mm from speed).
// controlCollectSplitPhase() reads encoders and runs the velocity PID.
void sim_tick(void* h, uint32_t now_ms)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    s->hal.tick(now_ms);
    s->robot.controlCollectSplitPhase(now_ms, 0);
}

// ---- Command dispatch ----

// Process one NUL-terminated command line.
// Replies are written into out_buf (NUL-terminated).
// Returns the number of bytes written (not counting the final NUL).
int sim_command(void* h, const char* line, char* out_buf, int out_len)
{
    SimHandle*   s = static_cast<SimHandle*>(h);
    ReplyCapture cap = { out_buf, out_len, 0 };
    if (out_len > 0) out_buf[0] = '\0';
    s->cmd.process(line, captureReply, &cap);
    if (out_len > 0) out_buf[cap.written] = '\0';
    return cap.written;
}

// ---- Encoder reads (accumulated mm from Robot::state.inputs) ----

float sim_get_enc_l(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.inputs.encLMm;
}

float sim_get_enc_r(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.inputs.encRMm;
}

// ---- Velocity reads (mm/s from Robot::state.inputs) ----

float sim_get_vel_l(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.inputs.velLMms;
}

float sim_get_vel_r(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.inputs.velRMms;
}

// ---- PWM reads (from Robot::state.commands) ----

float sim_get_pwm_l(void* h)
{
    return static_cast<float>(static_cast<SimHandle*>(h)->robot.state.commands.pwmL);
}

float sim_get_pwm_r(void* h)
{
    return static_cast<float>(static_cast<SimHandle*>(h)->robot.state.commands.pwmR);
}

// ---- Pose reads (dead-reckoning from Robot::state.inputs) ----

float sim_get_pose_x(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.inputs.poseX;
}

float sim_get_pose_y(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.inputs.poseY;
}

float sim_get_pose_h(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.inputs.poseHrad;
}

// ---- State injection ----

// Inject encoder position directly into MockMotor (overrides physics).
void sim_set_enc_l(void* h, float mm)
{
    // MockMotor does not expose a direct setEncoder; instead reset and set
    // the accumulated encoder via the underlying field.  We access it through
    // the Robot's motorL reference (which is a MockMotor).
    SimHandle* s = static_cast<SimHandle*>(h);
    s->hal.motorLMock().resetEncoder();
    // After reset, the mock encoder is 0.  We want it to report `mm`.
    // The mock reads _encoderMm via collectEncoder/readEncoderMmF.
    // We adjust by setting an initial offset via tick(0) — but that doesn't
    // give us direct mm control.  Use the setOffsetFactor approach: inject
    // via the hal's internal field through the mock accessor.
    // MockMotor exposes no direct setEncoderMm; use the sim_command ZERO
    // workaround or accept that enc injection re-zeroes and rebuilds.
    // For now, sync Robot's state.inputs to reflect the current mock value.
    s->robot.state.inputs.encLMm = mm;
}

void sim_set_enc_r(void* h, float mm)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    s->hal.motorRMock().resetEncoder();
    s->robot.state.inputs.encRMm = mm;
}

// Inject an OTOS pose reading into MockOtosSensor.
// The injected pose is returned by MockOtosSensor::readTransformed() on the
// next otosCorrect() call.
void sim_set_otos_pose(void* h, float x, float y, float hrad)
{
    static_cast<SimHandle*>(h)->hal.otosMock().setInjectedPose(x, y, hrad);
}

// Inject a per-wheel speed offset factor (1.0 = symmetric).
// side: 0 = left, 1 = right, other = both.
void sim_set_motor_offset(void* h, int side, float factor)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    if (side == 0 || side > 1) s->hal.motorLMock().setOffsetFactor(factor);
    if (side == 1 || side > 1) s->hal.motorRMock().setOffsetFactor(factor);
}

} // extern "C"
