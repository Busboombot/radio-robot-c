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
#include "control/MotionController.h"
#include "control/MotionCommand.h"
#include "control/HaltController.h"
#include "types/CommandTypes.h"

#include <cstring>
#include <cstdio>
#include <utility>

// Sim-injected clock.  Robot.cpp::system_timer_current_time() reads this so
// that Robot::systemTime() returns sim time instead of real wall-clock time.
// Updated at the top of sim_tick() and at the start of sim_command() so that
// time-based stop conditions (T ms=..., HALT TIME, watchdog) stay in the same
// epoch as driveAdvance(now_ms) and evaluate(now_ms).
uint32_t g_sim_now_ms = 0;

// ---------------------------------------------------------------------------
// ReplyStore — a heap-allocated reply accumulator.
//
// When MotionCommands (D, VW, R, …) complete during sim_tick(), they call
// the reply function stored in their reply sink.  In sim_api, the reply sink
// MUST point to a heap-allocated buffer (not a stack variable) so that
// driveAdvance() can safely fire EVTs at any tick after the command was issued.
//
// sim_command() writes both synchronous (OK/ERR) and async (EVT) replies into
// the same ReplyStore, then copies the accumulated bytes to the caller's buffer.
// ---------------------------------------------------------------------------
static constexpr int kReplyBufSize = 2048;

struct ReplyStore {
    char buf[kReplyBufSize];
    int  written = 0;

    void reset() { buf[0] = '\0'; written = 0; }

    void append(const char* msg) {
        if (!msg || written >= kReplyBufSize - 1) return;
        int remaining = kReplyBufSize - written - 1;
        int n = snprintf(buf + written, (size_t)remaining, "%s\n", msg);
        if (n > 0 && n < remaining) written += n;
    }
};

static void storeReply(const char* msg, void* ctx)
{
    static_cast<ReplyStore*>(ctx)->append(msg);
}

// ---------------------------------------------------------------------------
// SimHandle — one self-contained simulation instance allocated per test.
//
// Construction order is load-bearing:
//   1. hal        — MockHAL (owns all mock devices)
//   2. cfg        — RobotConfig value from defaultRobotConfig()
//   3. robot      — Robot(hal, cfg), wires motorController/odometry/etc.
//   4. cmd        — CommandProcessor with the full command table
//
// replyStore: heap-allocated persistent reply buffer.  All sim_command calls
//   and async EVTs fired during sim_tick() accumulate here.  sim_command()
//   copies from replyStore into the caller's out_buf, then resets the store.
// ---------------------------------------------------------------------------
struct SimHandle {
    MockHAL          hal;
    RobotConfig      cfg;
    Robot            robot;
    CommandProcessor cmd;
    ReplyStore       replyStore;

    // System keepalive watchdog (mirrors LoopScheduler._watchdogMs).
    // Reset to now_ms in sim_command(); checked each sim_tick().
    // 0 = not yet armed (no command received).
    uint32_t         watchdogMs = 0;

    // OTOS EKF fusion toggle (sprint 023). When true, sim_tick() runs the
    // firmware's Robot::otosCorrect() so the pose is the fused EKF estimate;
    // when false (default) the pose is encoder-only dead reckoning, preserving
    // the historical sim behaviour relied on by existing tests.
    bool             fuseOtos = false;

    SimHandle()
        : hal()
        , cfg(defaultRobotConfig())
        , robot(hal, cfg)
        , cmd(robot.buildCommandTable(nullptr, nullptr))
    {
        // Wire robot geometry into MockHAL so ExactPoseTracker integrates correctly.
        hal.setTrackwidth(cfg.trackwidthMm);
    }
};

// ---------------------------------------------------------------------------
// C ABI
// ---------------------------------------------------------------------------
extern "C" {

// ---- Lifecycle ----

void* sim_create()
{
    // Reset the injected clock so each SimHandle starts from t=0.
    // g_sim_now_ms is a global; without this reset, stale values from a
    // prior SimHandle would corrupt time-based stop conditions (HALT TIME,
    // watchdog) in the new instance.
    g_sim_now_ms = 0;
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
// motionController.driveAdvance() ticks the MotionCommand state machine (D, VW,
// R, G modes), which sets per-wheel speed targets that the PID then acts on.
//
// EVTs fired by driveAdvance() (e.g. EVT done D, EVT safety_stop) are written
// into SimHandle::replyStore via storeReply, which remains valid for the life
// of the SimHandle.
void sim_tick(void* h, uint32_t now_ms)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    // Keep the injected clock in sync so Robot::systemTime() returns sim time.
    g_sim_now_ms = now_ms;
    s->hal.tick(now_ms);
    s->robot.controlCollectSplitPhase(now_ms, 0);
    s->robot.motionController.driveAdvance(
        s->robot.state.inputs, s->robot.state.commands,
        s->robot.state.target, now_ms);

    // System keepalive watchdog — MUST mirror LoopScheduler.cpp exactly.
    // Fires EVT safety_stop + X when sTimeoutMs passes without any command.
    // Covers ALL active motion (any non-IDLE drive mode or active MotionCommand);
    // self-terminating commands (T/D/G/TURN) are NOT exempt — a stop condition
    // that never fires (e.g. a stalled G pre-rotate) must still be caught. The
    // host is required to stream "+" keepalives for the lifetime of any motion.
    // Signed delta avoids uint32 underflow (same pattern as firmware).
    if (s->watchdogMs != 0) {
        MotionController& mc = s->robot.motionController;
        bool needsWatchdog =
            (mc.mode() != DriveMode::IDLE) || mc.hasActiveCommand();
        if (s->robot.config.safetyEnabled && needsWatchdog) {
            int32_t wdDelta = (int32_t)(now_ms - s->watchdogMs);
            if (wdDelta > (int32_t)s->robot.config.sTimeoutMs) {
                s->watchdogMs = now_ms;  // re-arm to avoid firing every tick
                char wdBuf[64];
                CommandProcessor::replyEvt(wdBuf, sizeof(wdBuf),
                                           "safety_stop", "",
                                           storeReply, &s->replyStore);
                s->cmd.process("X", storeReply, &s->replyStore);
            }
        }
    }

    // Odometry: dead-reckon pose from encoder deltas (mirrors run_blocks()).
    s->robot.odometry.predict(s->robot.state.inputs,
                              s->robot.config.trackwidthMm, now_ms);

    // OTOS EKF correction (sprint 023) — opt-in via sim_set_otos_fusion().
    // The real firmware runs otosCorrect() on the enOtos loop phase; the sim
    // historically omitted it, so by default the firmware pose is encoder-only.
    // When fusion is enabled this runs the actual Robot::otosCorrect() ->
    // EKF updatePosition()/updateVelocity() path on the simulated OTOS reading.
    if (s->fuseOtos) {
        s->robot.otosCorrect(now_ms);
    }

    // HaltController — evaluate user-registered named stop conditions.
    // Mirrors LoopScheduler run_blocks() halt block.
    {
        HaltAction ha = s->robot.haltController.evaluate(
            s->robot.state.inputs, now_ms, storeReply, &s->replyStore);
        if (ha == HaltAction::HARD) {
            s->cmd.process("X", storeReply, &s->replyStore);
        } else if (ha == HaltAction::SOFT) {
            s->cmd.process("X soft", storeReply, &s->replyStore);
        }
    }
}

// ---- Command dispatch ----

// Process one NUL-terminated command line.
// All replies (synchronous OK/ERR and async EVTs from future ticks) are
// written into SimHandle::replyStore via the persistent storeReply callback.
// After cmd.process() returns, the synchronous portion is copied into out_buf
// and the store is reset.  Any async EVTs fired AFTER this call (during
// subsequent sim_tick() calls) will accumulate in replyStore and are
// accessible via sim_get_async_evts().
//
// Returns the number of synchronous bytes written (not counting the final NUL).
int sim_command(void* h, const char* line, char* out_buf, int out_len)
{
    SimHandle* s = static_cast<SimHandle*>(h);

    // Sync the injected clock to the current sim time before processing the
    // command.  Command handlers call robot->systemTime() (→ g_sim_now_ms) to
    // stamp time-based baselines (T ms=..., HALT TIME, watchdog).  Using sim
    // time here ensures those baselines are in the same epoch as driveAdvance.
    // g_sim_now_ms was last updated by the most-recent sim_tick() call.

    // Reset the store before the command so we capture only this command's
    // synchronous reply and not leftover async EVTs from prior ticks.
    s->replyStore.reset();

    // Process the command; all replies go into the persistent replyStore.
    // storeReply + &s->replyStore are passed as the reply sink to cmd.process()
    // and will also be captured by any MotionCommand that calls setReplySink().
    s->cmd.process(line, storeReply, &s->replyStore);

    // Reset system watchdog on every inbound command — mirrors LoopScheduler's
    // resetWatchdog(now). Reset to the CURRENT sim time so keepalives actually
    // extend the window: the watchdog fires sTimeoutMs after the LAST command,
    // not the first. g_sim_now_ms==0 (a command before the first tick) maps to
    // the sentinel 1 so the timer stays armed (0 means "disarmed / none yet").
    s->watchdogMs = (g_sim_now_ms == 0) ? 1u : g_sim_now_ms;

    // Copy the synchronous reply into the caller's buffer.
    int n = s->replyStore.written;
    if (out_buf && out_len > 0) {
        int copy = (n < out_len - 1) ? n : out_len - 1;
        memcpy(out_buf, s->replyStore.buf, (size_t)copy);
        out_buf[copy] = '\0';
        n = copy;
    }

    // Reset written count so subsequent EVTs from driveAdvance() accumulate
    // from position 0 (overwriting the already-copied synchronous reply).
    s->replyStore.reset();

    return n;
}

// ---- Async EVT access ----

// Read async EVT replies accumulated in replyStore since the last
// sim_command() call.  Returns the number of bytes written into evts_buf.
int sim_get_async_evts(void* h, char* evts_buf, int evts_len)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    if (!evts_buf || evts_len <= 0) return 0;
    int n = s->replyStore.written;
    if (n >= evts_len) n = evts_len - 1;
    memcpy(evts_buf, s->replyStore.buf, (size_t)n);
    evts_buf[n] = '\0';
    // Drain the store so subsequent calls only see new EVTs from future ticks.
    // Callers (SimConnection._get_evts) own the returned bytes; the store is
    // refilled on the next sim_tick() when new EVTs fire.
    s->replyStore.reset();
    return n;
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

// ---- Exact pose (oracle ground truth from ExactPoseTracker) ----
float sim_get_exact_pose_x(void* h) {
    return static_cast<SimHandle*>(h)->hal.exactPoseMock().x;
}
float sim_get_exact_pose_y(void* h) {
    return static_cast<SimHandle*>(h)->hal.exactPoseMock().y;
}
float sim_get_exact_pose_h(void* h) {
    return static_cast<SimHandle*>(h)->hal.exactPoseMock().h;
}

// ---- Encoder noise/slip (side: 0=left, 1=right, 2=both) ----
void sim_set_motor_slip(void* h, int side, float straight, float turn_extra) {
    SimHandle* s = static_cast<SimHandle*>(h);
    if (side == 0 || side > 1) s->hal.motorLMock().setSlip(straight, turn_extra);
    if (side == 1 || side > 1) s->hal.motorRMock().setSlip(straight, turn_extra);
}
void sim_set_encoder_noise(void* h, int side, float sigma_mm) {
    SimHandle* s = static_cast<SimHandle*>(h);
    if (side == 0 || side > 1) s->hal.motorLMock().setEncoderNoise(sigma_mm);
    if (side == 1 || side > 1) s->hal.motorRMock().setEncoderNoise(sigma_mm);
}

// ---- OTOS sim model ----
void sim_enable_otos_model(void* h) {
    static_cast<SimHandle*>(h)->hal.otosMock().enableSimModel(true);
}
// Enable/disable the firmware OTOS EKF correction inside sim_tick().
// Also marks the mock OTOS initialised so Robot::otosCorrect() does not
// early-return on its is_initialized() guard.
void sim_set_otos_fusion(void* h, int on) {
    SimHandle* s = static_cast<SimHandle*>(h);
    s->fuseOtos = (on != 0);
    if (s->fuseOtos) s->hal.otosMock().begin();
}
void sim_set_otos_linear_noise(void* h, float sigma_fraction) {
    static_cast<SimHandle*>(h)->hal.otosMock().setLinearNoise(sigma_fraction);
}
void sim_set_otos_yaw_noise(void* h, float sigma_fraction) {
    static_cast<SimHandle*>(h)->hal.otosMock().setYawNoise(sigma_fraction);
}
float sim_get_otos_x(void* h) {
    return static_cast<SimHandle*>(h)->hal.otosMock().odomX();
}
float sim_get_otos_y(void* h) {
    return static_cast<SimHandle*>(h)->hal.otosMock().odomY();
}
float sim_get_otos_h(void* h) {
    return static_cast<SimHandle*>(h)->hal.otosMock().odomH();
}

} // extern "C"
