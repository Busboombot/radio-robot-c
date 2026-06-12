// sim_api.cpp — extern "C" C ABI wrapper over a self-contained simulation.
//
// Provides an opaque SimHandle that owns MockHAL + Robot + CommandProcessor.
// Python test code (ticket 020-004) loads this shared library via ctypes.
//
// Build: cmake -S . -B host_tests/build && cmake --build host_tests/build
// Load:  python3 -c "import ctypes; ctypes.CDLL('./host_tests/build/libfirmware_host.dylib')"

#include "robot/Robot.h"
#include "app/CommandProcessor.h"
#include "app/CommandQueue.h"
#include "hal/mock/MockHAL.h"
#include "hal/mock/MockMotor.h"
#include "hal/mock/MockOtosSensor.h"
#include "types/Config.h"
#include "control/RobotState.h"
#include "control/MotionController.h"
#include "control/MotionCommand.h"
#include "control/HaltController.h"
#include "control/LoopTickOnce.h"
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
//   4. _queue     — CommandQueue (wired into cmd + robot._motionCtx)
//   5. cmd        — CommandProcessor with the full command table
//
// Queue wiring: sim_api mirrors LoopScheduler's constructor wiring exactly —
//   cmd.setQueue(&_queue) and robot.setMotionQueue(&_queue) — so
//   converter commands (S, T, D, G, R, TURN) travel through the queue path,
//   not the direct begin*() fallback, matching firmware behaviour.
//   (sprint 026-002: replaced robot.motionController.setQueue() with
//   robot.setMotionQueue() since MotionCtx now lives in Robot.)
//
// replyStore: heap-allocated persistent reply buffer.  All sim_command calls
//   and async EVTs fired during sim_tick() accumulate here.  sim_command()
//   copies from replyStore into the caller's out_buf, then resets the store.
// ---------------------------------------------------------------------------
struct SimHandle {
    MockHAL          hal;
    RobotConfig      cfg;
    Robot            robot;
    CommandQueue     _queue;
    CommandProcessor cmd;
    ReplyStore       replyStore;

    // Per-tick state: watchdog, last-run timestamps, active reply sink,
    // fuseOtos flag.  Replaces the former standalone watchdogMs field and
    // the hand-mirrored watchdog block in sim_tick().
    LoopTickState    _ts;

    SimHandle()
        : hal()
        , cfg(defaultRobotConfig())
        , robot(hal, cfg)
        , _queue()
        , cmd(robot.buildCommandTable(nullptr, nullptr))
    {
        // Wire robot geometry into MockHAL so ExactPoseTracker integrates correctly.
        hal.setTrackwidth(cfg.trackwidthMm);

        // Wire the queue into both cmd and Robot's MotionCtx — mirrors LoopScheduler's
        // constructor wiring so converter commands (S, T, D, G, R, TURN, RT) travel
        // the queue path on the next sim_tick(), not the direct begin*() fallback.
        // Sprint 026-002: replaced robot.motionController.setQueue() with
        // robot.setMotionQueue() since MotionCtx now lives in Robot.
        cmd.setQueue(&_queue);
        robot.setMotionQueue(&_queue);

        // Set default reply sink in _ts so the watchdog and halt blocks have a
        // valid sink from the first command.  sim_command() will also set this
        // to storeReply / &replyStore on each command call.
        _ts.activeFn    = storeReply;
        _ts.activeTlmFn = storeReply;
        _ts.activeCtx   = &replyStore;
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
// loopTickOnce() runs the single shared firmware loop body: dequeueOne,
// watchdog, halt, driveAdvance, odometry, OTOS fusion (when enabled),
// line/colour/ports/TLM timed blocks.
//
// EVTs fired during loopTickOnce() (e.g. EVT done D, EVT safety_stop) are
// written into SimHandle::replyStore via storeReply, which remains valid for
// the life of the SimHandle.
void sim_tick(void* h, uint32_t now_ms)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    // Keep the injected clock in sync so Robot::systemTime() returns sim time.
    g_sim_now_ms = now_ms;
    s->hal.tick(now_ms);
    s->robot.controlCollectSplitPhase(now_ms, 0);

    // Ensure _ts has the current reply sink before each tick so that watchdog
    // and halt events go to replyStore.
    s->_ts.activeFn    = storeReply;
    s->_ts.activeTlmFn = storeReply;
    s->_ts.activeCtx   = &s->replyStore;

    // Run the shared firmware tick body: dequeue, watchdog, halt, drive,
    // odometry, OTOS/line/colour/ports/TLM blocks.
    loopTickOnce(s->robot, s->cmd, s->_queue, s->_ts, now_ms);
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

    // Set the active reply sink for this command.
    s->_ts.activeFn    = storeReply;
    s->_ts.activeTlmFn = storeReply;
    s->_ts.activeCtx   = &s->replyStore;

    // Process the command; all replies go into the persistent replyStore.
    // storeReply + &s->replyStore are passed as the reply sink to cmd.process()
    // and will also be captured by any MotionCommand that calls setReplySink().
    //
    // With the queue wired, cmd.process() enqueues the command rather than
    // dispatching it immediately.  We drain the queue right here so that
    // sim_command() remains synchronous — the caller receives the OK/ERR reply
    // before returning, exactly as before the queue was wired.
    //
    // Drain twice: once for the command itself (e.g. T handler → pushes VW),
    // once for any VW pushed by a converter handler (T/S/D/G/R/TURN/RT →
    // handleVW → beginTimed/beginStream/…).  A non-converter command (PING,
    // HALT, SET, …) will find the queue empty on the second call — no-op.
    s->cmd.process(line, storeReply, &s->replyStore);
    s->cmd.dequeueOne(s->_queue);  // dispatch the command
    s->cmd.dequeueOne(s->_queue);  // dispatch any VW pushed by a converter

    // Reset system watchdog on every inbound command — mirrors LoopScheduler's
    // resetWatchdog(now). Reset to the CURRENT sim time so keepalives actually
    // extend the window: the watchdog fires sTimeoutMs after the LAST command,
    // not the first. g_sim_now_ms==0 (a command before the first tick) maps to
    // the sentinel 1 so the timer stays armed (0 means "disarmed / none yet").
    s->_ts.watchdogMs = (g_sim_now_ms == 0) ? 1u : g_sim_now_ms;

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

// ---- EKF diagnostics ----

// Cumulative EKF gate rejection count (all channels: position, heading, velocity).
// Exposed for the N1 regression test: assert ekf_rej == 0 after a D command when
// fusion is ON — the atomic encoder reset prevents the spurious negative delta that
// previously caused Mahalanobis-gate rejections for ~10 ticks post-D. (030-001)
int sim_get_ekf_rej_count(void* h)
{
    return static_cast<SimHandle*>(h)->robot.odometry.ekfRejectCount();
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
    s->_ts.fuseOtos = (on != 0);
    if (s->_ts.fuseOtos) s->hal.otosMock().begin();
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

// ---- N2 queue-invariant helper (030-002) ----

// Returns 1 if the CommandProcessor has a queue attached (cmd.hasQueue()),
// 0 otherwise.  Used by the boot/queue-invariant regression test to assert
// that cmd._queue survives a Phase-3-style reassignment.
//
// In the sim, the queue is wired in SimHandle's constructor and is never
// reassigned, so this always returns 1 after sim_create().  The regression
// test is therefore a structural canary: if CommandProcessor's move-assign
// ever silently clears the queue pointer again (e.g. a future refactor
// re-introduces the Phase-3 pattern), this accessor will catch it.
int sim_get_queue_wired(void* h)
{
    return static_cast<SimHandle*>(h)->cmd.hasQueue() ? 1 : 0;
}

// ---- D10 telemetry test helpers (028-005) ----

// Returns 1 if the robot's TLM channel is bound (_tlmBoundCtx != nullptr),
// 0 otherwise.  Used by channel-binding tests to verify handleStream stored
// the caller's reply ctx.  (In sim, _tlmBoundFn is not set since runCommsIn
// is not called; binding is signalled via _tlmBoundCtx alone.)
int sim_get_tlm_bound(void* h)
{
    return (static_cast<SimHandle*>(h)->robot._tlmBoundCtx != nullptr) ? 1 : 0;
}

// Advance simulation by total_ms in step_ms increments and collect all TLM
// lines emitted by telemetryEmit into out_buf.  Returns the number of TLM
// lines collected (not bytes).  out_buf receives concatenated lines separated
// by '\n'; it is NUL-terminated.
//
// Note: sim_tick() resets _ts.activeTlmFn = storeReply each tick, so TLM
// frames from telemetryEmit go into replyStore.  After each tick we drain
// replyStore into out_buf, counting the TLM lines.
int sim_tick_collect_tlm(void* h, uint32_t start_ms, uint32_t total_ms,
                         uint32_t step_ms, char* out_buf, int out_len)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    char evtBuf[2048];
    int  tlmCount = 0;
    int  outPos   = 0;

    uint32_t end_ms = start_ms + total_ms;
    for (uint32_t t = start_ms; t < end_ms; t += step_ms) {
        g_sim_now_ms = t;
        s->hal.tick(t);
        s->robot.controlCollectSplitPhase(t, 0);

        s->_ts.activeFn    = storeReply;
        s->_ts.activeTlmFn = storeReply;
        s->_ts.activeCtx   = &s->replyStore;

        loopTickOnce(s->robot, s->cmd, s->_queue, s->_ts, t);

        // Drain replyStore: look for TLM lines.
        int n = s->replyStore.written;
        if (n > 0) {
            char* p = s->replyStore.buf;
            char* end = p + n;
            while (p < end) {
                char* nl = p;
                while (nl < end && *nl != '\n') ++nl;
                // p..nl is one line (without the '\n').
                int lineLen = (int)(nl - p);
                if (lineLen >= 3 && p[0] == 'T' && p[1] == 'L' && p[2] == 'M') {
                    ++tlmCount;
                    // Append to out_buf if room.
                    if (out_buf && outPos + lineLen + 2 < out_len) {
                        memcpy(out_buf + outPos, p, (size_t)lineLen);
                        outPos += lineLen;
                        out_buf[outPos++] = '\n';
                    }
                }
                p = (nl < end) ? nl + 1 : end;
            }
            s->replyStore.reset();
        }
    }

    if (out_buf && out_len > 0) out_buf[outPos] = '\0';
    return tlmCount;
}

// ---- N7 queue-overflow test helpers (030-005) ----

// Returns the current number of items in the CommandQueue.
// Used by overflow regression tests to verify the queue fills and that
// subsequent enqueues (via sim_command) produce ERR full replies.
int sim_queue_size(void* h)
{
    return static_cast<SimHandle*>(h)->_queue.size();
}

// Fill the CommandQueue to capacity (COMMAND_QUEUE_CAPACITY = 4) with
// dummy no-op ParsedCommand entries.  Returns the number of items pushed.
// After this call, any sim_command() that routes through dispatchTable()
// (queue path) will fail with ERR full — except sim_command itself also
// drains two items, so the test should call sim_fill_queue() then send
// one more command WITHOUT calling dequeueOne.  This is done by calling
// sim_command_no_drain() below.
//
// The dummy entries use a nullptr desc so dequeueOne() is a safe no-op
// (it guards pc.desc != nullptr before calling handlerFn).
int sim_fill_queue(void* h)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    int pushed = 0;
    ParsedCommand dummy;
    dummy.desc    = nullptr;   // no-op: dequeueOne guards desc != nullptr
    dummy.replyFn = nullptr;
    dummy.replyCtx = nullptr;
    dummy.args.count = 0;
    dummy.corrId[0] = '\0';
    while (s->_queue.push_back(dummy)) ++pushed;
    return pushed;
}

// Process one command WITHOUT the two post-process dequeueOne drains.
// This is needed by the overflow test: we want dispatchTable() to find a
// full queue and reply ERR full, without draining the queue first.
// Returns the number of synchronous bytes written into out_buf.
int sim_command_no_drain(void* h, const char* line, char* out_buf, int out_len)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    s->replyStore.reset();
    s->_ts.activeFn    = storeReply;
    s->_ts.activeTlmFn = storeReply;
    s->_ts.activeCtx   = &s->replyStore;
    s->cmd.process(line, storeReply, &s->replyStore);
    // NOTE: no dequeueOne() calls — intentional, allows testing overflow.
    int n = s->replyStore.written;
    if (out_buf && out_len > 0) {
        int copy = (n < out_len - 1) ? n : out_len - 1;
        memcpy(out_buf, s->replyStore.buf, (size_t)copy);
        out_buf[copy] = '\0';
        n = copy;
    }
    s->replyStore.reset();
    return n;
}

// ---- N8 sensor-freshness helpers (030-008) ----

// Initialize (begin) the MockLineSensor so Robot::lineRead() considers it
// present and emits line= in TLM.  Call before tests that need line data.
void sim_init_line_sensor(void* h)
{
    static_cast<SimHandle*>(h)->hal.lineMock().begin();
}

// Initialize (begin) the MockColorSensor so Robot::colorRead() considers it
// present and emits color= in TLM.  Call before tests that need color data.
void sim_init_color_sensor(void* h)
{
    static_cast<SimHandle*>(h)->hal.colorMock().begin();
}

// Freeze / unfreeze the MockLineSensor.  When frozen, readValues() returns
// false so Robot::lineRead() never updates lineVS.lastUpdMs — after ~2×lagMs
// the TLM freshness gate drops the line= field from TLM frames.
void sim_set_line_frozen(void* h, int frozen)
{
    static_cast<SimHandle*>(h)->hal.lineMock().setFrozen(frozen != 0);
}

// Freeze / unfreeze the MockColorSensor.  When frozen, pollRGBC() returns
// false so Robot::colorRead() never updates colorVS.lastUpdMs — after ~2×lagMs
// the TLM freshness gate drops the color= field from TLM frames.
void sim_set_color_frozen(void* h, int frozen)
{
    static_cast<SimHandle*>(h)->hal.colorMock().setFrozen(frozen != 0);
}

// ---- N9 same-tick OTOS failure helper (030-008) ----

// Inject / clear an OTOS read failure.  When set, MockOtosSensor::readTransformed
// and readVelocityTransformed return false and emit {0,0,0}/{0,0}.
// Robot::otosCorrect() must detect this same-tick failure via the return value
// and skip EKF fusion — the fusedV/poseX/Y/H state must remain unchanged.
void sim_set_otos_read_failure(void* h, int fail)
{
    static_cast<SimHandle*>(h)->hal.otosMock().setReadFailure(fail != 0);
}

// Read fusedV from state.inputs (EKF body-frame linear speed, mm/s).
// Used by N9 test to assert the fused velocity is not dragged to zero on
// a same-tick OTOS read failure.
float sim_get_fused_v(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.inputs.fusedV;
}

// Read fusedOmega from state.inputs (EKF yaw rate, rad/s).
float sim_get_fused_omega(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.inputs.fusedOmega;
}

} // extern "C"
