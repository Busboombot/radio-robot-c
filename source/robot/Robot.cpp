#include "Robot.h"
#include "MotionController.h"
#ifndef HOST_BUILD
#include "MicroBit.h"
#include "MicroBitDevice.h"
#include "LoopScheduler.h"
#include "Communicator.h"
#include "Radio.h"
#include "RadioChannel.h"
#endif
#include "Odometry.h"
#include "DebugCommandable.h"
#include "CommandProcessor.h"
#include "ConfigRegistry.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cassert>

// ---------------------------------------------------------------------------
// HOST_BUILD stubs — replace CODAL runtime calls with safe no-op equivalents.
// These are only compiled when building the shared library for host tests.
// ---------------------------------------------------------------------------
#ifdef HOST_BUILD
#include <cstdint>

// Sim-injected clock — updated by sim_tick() and sim_command() in sim_api.cpp
// so that Robot::systemTime() returns sim time rather than real wall-clock time.
// This ensures time-based stop conditions (T, HALT TIME) use the same epoch as
// driveAdvance(now_ms) and evaluate(now_ms), preventing immediate false-fire.
extern uint32_t g_sim_now_ms;

static const char* microbit_friendly_name() { return "sim"; }
static uint32_t    microbit_serial_number()  { return 0; }
static uint32_t    system_timer_current_time() { return g_sim_now_ms; }
#endif

// ---------------------------------------------------------------------------
// Constructor — initializer list must match member declaration order.
//
// Declaration order (from Robot.h):
//   hal, config, state, motorL, motorR, otos, line, colorSensor, gripper, portio,
//   motorController, odometry, motionController, portController, servoController
//
// hal must be declared (and therefore initialized) before the interface refs so
// that hal.motorL() etc. are valid when the refs are bound.
//
// Two post-construction binds:
//   motionController.setHardwareState(&state.inputs)  — MotionController reads pose
//   motorController.setCommandsRef(&state.commands)   — MotorController writes tgt*/pwm*
// ---------------------------------------------------------------------------

Robot::Robot(Hardware& h, const RobotConfig& cfg)
    : hal(h),
      config(cfg),
      state(defaultInputs(cfg)),
      motorL(hal.motorL()), motorR(hal.motorR()),
      otos(hal.otos()), line(hal.lineSensor()),
      colorSensor(hal.colorSensor()), gripper(hal.gripper()), portio(hal.portIO()),
      motorController(motorL, motorR, config),
      odometry(),
      motionController(motorController, odometry, config),
      portController(portio),
      servoController(gripper)
{
    motionController.setHardwareState(&state.inputs);
    motorController.setCommandsRef(&state.commands);
    // setRobotCtx replaces setCtx (sprint 026-002): MotionCtx now lives in Robot.
    motionController.setRobotCtx(this);
    // Initialise _motionCtx (sprint 026-002): mc and robot pointers; queue wired
    // later by setMotionQueue() from LoopScheduler or test harness.
    _motionCtx.mc    = &motionController;
    _motionCtx.robot = this;
    _motionCtx.queue = nullptr;
    odometry.setCtx(&otos, &state.inputs);
    odometry.initEKF(config.ekfQxy, config.ekfQtheta,
                     config.ekfQv, config.ekfQomega,
                     config.ekfROtosXy, config.ekfROtosV, config.ekfREncV,
                     config.ekfROtosTheta);
}

// ---------------------------------------------------------------------------
// systemTime — robot system time in milliseconds since boot.
// ---------------------------------------------------------------------------

uint32_t Robot::systemTime() const
{
    return (uint32_t)system_timer_current_time();
}

// ---------------------------------------------------------------------------
// controlCollectSplitPhase — split-phase COLLECT for the cooperative loop.
//
// Reads both encoders, applies the speed-scaled outlier filter, writes
// state.inputs.enc{L,R}Mm, then calls motorController.controlTick() for PID+PWM.
//
// Migrated from the original Robot controlCollectSplitPhase with mechanical
// member-name substitutions (_state → state, _mc → motorController,
// _motorL → motorL, _motorR → motorR, _config → config).
// ---------------------------------------------------------------------------

void Robot::controlCollectSplitPhase(uint32_t now_ms, int /*pendingWheel*/)
{
    // WedgeTest-proven pattern (sprint 015): read BOTH encoders every tick,
    // right motor (M1) first, then left (M2). Write-on-change is already
    // handled by Motor::setSpeed(). Single re-read on implausible delta.
    //
    // Cost: ~8 ms (2 × 4 ms post-write settle). controlPeriodMs must be ≥ 10 ms.
    //
    // Previous alternating-one-per-tick design (~5 Hz per wheel) wedged within
    // ~165 ticks: each wedge caused the velocity PID to saturate and jerk.
    // WedgeTest ran 10 min / 165 cycles with ZERO wedges using this pattern.
    bool driving = (state.commands.tgtLMms != 0.0f ||
                    state.commands.tgtRMms != 0.0f);
    if (driving) {
        // Outlier threshold SCALES with commanded speed. A legit tick can't move
        // much more than (target speed × a worst-case ~200 ms scheduler tick), so
        // the gate is max(40 mm floor, |target mm/s| × 0.2). A bad read triggers up
        // to kRetries re-reads; if any is sane → use it; if ALL fail → hold the old
        // stored value so the outlier baseline stays correct next tick.
        //
        // Why scaled, not a fixed 150 mm: at slow calibration speeds (~80 mm/s) a
        // legit tick is <10 mm, but the chip still occasionally returns ~149 mm
        // garbage reads — which slipped UNDER a fixed 150 mm gate, fed the velocity
        // loop a huge spurious velocity, and spasmed the motor. Scaling keeps the
        // gate tight when slow (rejects those) and wide when fast (~80 mm at
        // 400 mm/s) so normal fast driving isn't tripped.
        const float kMaxDeltaMm = fmaxf(40.0f,
            fmaxf(fabsf((float)state.commands.tgtLMms),
                  fabsf((float)state.commands.tgtRMms)) * 0.2f);
        static constexpr int kRetries = 2;

        // Right (M1) first — proven ordering from WedgeTest.
        {
            float newR = motorR.readEncoderMmFSettle(config);
            float dR   = newR - state.inputs.encRMm;
            if (dR > kMaxDeltaMm || dR < -kMaxDeltaMm) {
                newR = state.inputs.encRMm;             // default: hold old
                for (int k = 0; k < kRetries; ++k) {
                    float r2  = motorR.readEncoderMmFSettle(config);
                    float dr2 = r2 - state.inputs.encRMm;
                    if (dr2 <= kMaxDeltaMm && dr2 >= -kMaxDeltaMm) { newR = r2; break; }
                }
            }
            state.inputs.encRMm = newR;
        }

        // Left (M2) second.
        {
            float newL = motorL.readEncoderMmFSettle(config);
            float dL   = newL - state.inputs.encLMm;
            if (dL > kMaxDeltaMm || dL < -kMaxDeltaMm) {
                newL = state.inputs.encLMm;             // default: hold old
                for (int k = 0; k < kRetries; ++k) {
                    float r2  = motorL.readEncoderMmFSettle(config);
                    float dr2 = r2 - state.inputs.encLMm;
                    if (dr2 <= kMaxDeltaMm && dr2 >= -kMaxDeltaMm) { newL = r2; break; }
                }
            }
            state.inputs.encLMm = newL;
        }
    }
    _prevDriving = driving;
    _lastControlMs = now_ms;
    // refreshedWheel=3: both wheels updated; 0: idle, no velocity update.
    motorController.controlTick(state.inputs, state.commands, now_ms, driving ? 3 : 0);
}

// ---------------------------------------------------------------------------
// otosCorrect — EKF Kalman update from OTOS position and velocity (sprint 023).
//
// Reads OTOS position, velocity, and acceleration.  Passes position + velocity
// to correctEKF() for EKF fusion.  Stores acceleration in HardwareState for
// host telemetry via RobotState.
//
// Encoder-rate velocity (enc_v, enc_omega) is retrieved from the most recent
// predict() call via Odometry::lastEncV()/lastEncOmega().  Design choice:
// these are stored on Odometry rather than threaded through the cooperative
// loop caller because predict() and otosCorrect() run on different loop phases
// (enOdom vs enOtos), so passing them through the caller would require
// HardwareState fields or Robot members anyway — no fewer coupling points.
// Storing them on Odometry keeps the call sites unchanged.
// ---------------------------------------------------------------------------

void Robot::otosCorrect(uint32_t now_ms)
{
    // Indirection through hal.otos() — reads the LIVE active pointer, not the
    // cached `otos` ref (which was bound at construction to the real OtosSensor
    // and cannot be re-seated).  When NezhaHAL::setOtosBench(true) is called,
    // hal.otos() returns the BenchOtosSensor; the cached `otos` ref keeps
    // pointing to the real chip.  This is the ONLY place otosCorrect() diverges
    // from the `otos` ref; all other Robot read sites keep the cached ref.
    // (sprint 031-002 reference-reseating fix)
    IOtosSensor& activeOtos = hal.otos();

    if (!activeOtos.is_initialized()) return;

    // -----------------------------------------------------------------------
    // D9 (027-005): OTOS STATUS register validity gate.
    //
    // Read REG_STATUS (0x1F) and check the most-recent I2C read flag before
    // fusing into the EKF.  A lifted or just-placed robot reports a non-zero
    // STATUS byte (tracking invalid).  Passing zero velocity to the EKF while
    // the sensor is invalid drags fused velocity to zero and fights the
    // controller — this was the root cause of the "spin on placement" symptom.
    //
    // EVT path (Open Question 3 resolution): we call
    //   motionController.emitToActiveChannel("EVT otos lost", state.target)
    // which wraps the existing static emitEvt(base, TargetState&) helper.
    // Robot owns state.target and can pass it directly; no new reply-sink
    // plumbing is required.  emitEvt routes via target.sink.emitFn — the
    // reply channel captured when the active command (G/T/D/TURN) started.
    // -----------------------------------------------------------------------
    uint8_t otosStatus = 0;
    bool statusOk = activeOtos.readStatus(otosStatus);

    if (!statusOk || otosStatus != 0 || !activeOtos.lastReadOk()) {
        // OTOS is invalid: do not fuse; mark the validity envelope.
        state.inputs.otos.valid = false;

        // Emit "EVT otos lost" exactly once per invalidity window,
        // but only when a motion command is actively running (no point
        // signalling on a parked robot).
        if (motionController.hasActiveCommand()) {
            if (_otosInvalidStartMs == 0) {
                _otosInvalidStartMs = now_ms;
            }
            if (!_otosLostEmitted &&
                ((now_ms - _otosInvalidStartMs) >= 500u)) {
                motionController.emitToActiveChannel("EVT otos lost",
                                                     state.target);
                _otosLostEmitted = true;
            }
        }
        return;
    }

    // OTOS valid: reset the invalidity tracking window.
    _otosInvalidStartMs = 0;
    _otosLostEmitted    = false;
    state.inputs.otos.valid = true;

    // Pass poseHrad for the lever-arm offset rotation (no-op when offsets are zero).
    float headingRad = state.inputs.poseHrad;

    // N9 (030-008): use the return value of readTransformed — NOT the stale
    // lastReadOk() from the previous tick.  _lastReadOk is updated INSIDE
    // readTransformed (by readXYH), so checking it before the call would miss
    // a failure that occurs on THIS tick.  A failed read decodes raw[6]={0}
    // into pose(0,0,0)/vel(0,0); near the origin the Mahalanobis gate accepts
    // these zeros and drags fusedV to zero — the D9 one-tick symptom.
    OtosPose p;
    bool poseOk = activeOtos.readTransformed(config, p, headingRad);
    if (!poseOk) {
        // Same-tick I2C failure: mark otos invalid and skip fusion.
        // Do not update otosX/Y/H or lastUpdMs with garbage zeros.
        state.inputs.otos.valid = false;
        return;
    }
    state.inputs.otosX = p.x;
    state.inputs.otosY = p.y;
    state.inputs.otosH = p.h;
    state.inputs.otos.lastUpdMs = now_ms;

    // Read OTOS velocity and acceleration; store acceleration for telemetry.
    OtosVelocity vel;
    bool velOk = activeOtos.readVelocityTransformed(config, vel, headingRad);
    OtosAccel    acc = activeOtos.readAccelTransformed(config);
    state.inputs.otosAccelX = acc.ax_mmps2;
    state.inputs.otosAccelY = acc.ay_mmps2;

    // If the velocity read also failed this tick, use zero velocity rather
    // than fusing garbage — the EKF's encoder-based velocity estimate is a
    // better fallback.  We still fuse pose (poseOk was true).
    if (!velOk) {
        vel.v_mmps     = 0.0f;
        vel.omega_rads = 0.0f;
    }

    // Retrieve encoder-rate velocity from the most recent predict() tick.
    float enc_v     = odometry.lastEncV();
    float enc_omega = odometry.lastEncOmega();

    odometry.correctEKF(state.inputs, p.x, p.y,
                        p.h,
                        vel.v_mmps, vel.omega_rads,
                        enc_v, enc_omega);
}

// ---------------------------------------------------------------------------
// lineRead — read 4-channel line sensor into HardwareState.
// ---------------------------------------------------------------------------

void Robot::lineRead()
{
    if (!line.is_initialized()) return;
    if (line.readValues(state.inputs.line)) {
        state.inputs.lineVS.lastUpdMs = systemTime();
        state.inputs.lineVS.valid     = true;
    }
}

// ---------------------------------------------------------------------------
// colorRead — non-blocking RGBC poll into HardwareState.
// ---------------------------------------------------------------------------

void Robot::colorRead()
{
    if (!colorSensor.is_initialized()) return;
    if (colorSensor.pollRGBC(state.inputs.colorR,
                              state.inputs.colorG,
                              state.inputs.colorB,
                              state.inputs.colorC)) {
        state.inputs.colorVS.lastUpdMs = systemTime();
        state.inputs.colorVS.valid     = true;
    }
}

// ---------------------------------------------------------------------------
// portsRead — read digital and analogue GPIO ports into HardwareState.
// ---------------------------------------------------------------------------

void Robot::portsRead()
{
    for (uint8_t i = 0; i < 4; ++i) {
        state.inputs.digitalIn[i] = (portio.readDigital(i) != 0);
        state.inputs.analogIn[i]  = (int16_t)portio.readAnalog(i);
    }
    state.inputs.portsVS.lastUpdMs = systemTime();
    state.inputs.portsVS.valid     = true;
}

// ---------------------------------------------------------------------------
// resetEncoders — single canonical atomic encoder reset (N1, sprint 030-001).
//
// Atomically resets hardware accumulators, MotorController velocity baselines,
// the outlier-filter baseline (state.inputs.encLMm/R), and Odometry's internal
// encoder snapshot — without touching pose.
//
// Previously distanceDrive() reset hardware+MC but left Odometry::_prevEncL/R
// stale, so the very next predict() computed dL = 0 - _prevEncL (large negative)
// and teleported the pose backward by the prior segment's travel.  ZERO enc
// was worse: hardware+MC reset but state.inputs.encLMm/R stayed stale, causing
// the outlier filter to freeze encoder reads until the fresh accumulator climbed
// back, then a pose jump.
// ---------------------------------------------------------------------------

void Robot::resetEncoders()
{
    // 1. Reset hardware accumulators AND MotorController velocity baselines
    //    (_prevEncL/R, _hasTimestamp*, _prevTimeMsL/R).
    motorController.resetEncoderAccumulators();

    // 2. Align the outlier-filter baseline with the now-zeroed accumulators.
    state.inputs.encLMm = 0.0f;
    state.inputs.encRMm = 0.0f;

    // 3. Re-baseline Odometry's encoder snapshot so predict() sees delta=0
    //    on the very next tick rather than (0 - _prevEncL) = large negative.
    odometry.rebaselinePrev(0.0f, 0.0f);
}

// ---------------------------------------------------------------------------
// benchOtosTick — feed commanded velocity into BenchOtosSensor each tick.
//
// Called from LoopTickOnce immediately before the OTOS block so that when
// bench mode is active, BenchOtosSensor::tick() integrates this tick's
// commanded velocities before otosCorrect() calls readTransformed().
//
// The dt computation uses a signed delta to avoid uint32 underflow
// (project memory: watchdog-uint32-underflow).  First call always passes
// dt=0 (no-op in BenchOtosSensor::tick).
//
// HOST_BUILD guard: NezhaHAL includes CODAL headers so is excluded from
// host builds.  The downcast to NezhaHAL* is wrapped in #ifndef HOST_BUILD;
// MockHAL returns nullptr and the function returns immediately, ensuring
// the host test suite is unaffected.
// ---------------------------------------------------------------------------

#ifndef HOST_BUILD
#include "NezhaHAL.h"
#endif

void Robot::benchOtosTick(uint32_t now_ms)
{
    // Signed-delta dt to avoid uint32 underflow (never plain-subtract two
    // uint32 ms stamps — project memory: watchdog-uint32-underflow).
    int32_t dt_signed = (int32_t)(now_ms - _lastBenchTickMs);
    uint32_t dt_ms = (dt_signed > 0) ? (uint32_t)dt_signed : 0u;
    _lastBenchTickMs = now_ms;

#ifndef HOST_BUILD
    // Downcast to NezhaHAL to access the active-pointer and bench sensor.
    // If hal is not a NezhaHAL (e.g. MockHAL in a hypothetical shared context),
    // return immediately — safe no-op.
    auto* nh = static_cast<NezhaHAL*>(&hal);

    // Early-return when bench mode is off (production path — nearly free).
    if (!nh->isBenchMode()) return;

    nh->benchOtosPtr()->tick(
        state.commands.tgtLMms,
        state.commands.tgtRMms,
        config.trackwidthMm,
        dt_ms);
#else
    // HOST_BUILD: NezhaHAL not available; bench mode always off in sim.
    (void)dt_ms;
#endif
}

// ---------------------------------------------------------------------------
// isBenchOtosActive — returns true when NezhaHAL has the bench sensor active.
// Always returns false in HOST_BUILD (MockHAL path).
// ---------------------------------------------------------------------------

bool Robot::isBenchOtosActive() const
{
#ifndef HOST_BUILD
    auto* nh = static_cast<const NezhaHAL*>(&hal);
    return nh->isBenchMode();
#else
    return false;
#endif
}

// ---------------------------------------------------------------------------
// distanceDrive — begin a distance drive and atomically reset encoder state.
// ---------------------------------------------------------------------------

void Robot::distanceDrive(int32_t l, int32_t r, int32_t targetMm,
                                ReplyFn fn, void* ctx, const char* corr_id)
{
    motionController.beginDistance((float)l, (float)r, targetMm,
                                   systemTime(), state.target, fn, ctx, corr_id);
    // Atomic encoder reset: aligns hardware accumulators, MC velocity baselines,
    // outlier-filter baseline, and Odometry encoder snapshot in one call.
    // (Replaces the split reset that was here + inside beginDistance().)
    resetEncoders();
}

// ---------------------------------------------------------------------------
// buildTlmFrame — assemble the unified TLM frame; returns length.
//
// Reads state.inputs, config, motionController.mode(). Shared by the periodic
// STREAM (telemetryEmit) and the synchronous SNAP command.
// ---------------------------------------------------------------------------

int Robot::buildTlmFrame(char* buf, int len)
{
    uint32_t t_sample = systemTime();
    int32_t encL = static_cast<int32_t>(state.inputs.encLMm);
    int32_t encR = static_cast<int32_t>(state.inputs.encRMm);

    int32_t pose_x = 0, pose_y = 0, pose_h = 0;
    if (config.tlmFields & TLM_FIELD_POSE) {
        Odometry::getPose(state.inputs, pose_x, pose_y, pose_h);
    }
    // N8 (030-008): gate line/color on freshness, not just the sticky valid bit.
    // A sensor that wedges after boot keeps valid=true forever; consult the
    // lastUpdMs / lagMs envelope instead: fresh = now − lastUpdMs ≤ 2×lagMs.
    // lagMs is 0 until the first valid read (last­UpdMs stays 0 too), so the
    // sub­traction wraps and the gate is never met — correct for "never read".
    bool haveLine = line.is_initialized() &&
                    state.inputs.lineVS.valid &&
                    (t_sample - state.inputs.lineVS.lastUpdMs
                         <= 2u * state.inputs.lineVS.lagMs) &&
                    (config.tlmFields & TLM_FIELD_LINE);
    bool haveColor = colorSensor.is_initialized() &&
                     state.inputs.colorVS.valid &&
                     (t_sample - state.inputs.colorVS.lastUpdMs
                          <= 2u * state.inputs.colorVS.lagMs) &&
                     (config.tlmFields & TLM_FIELD_COLOR);
    bool haveVel = (config.tlmFields & TLM_FIELD_VEL) != 0;
    float velL = haveVel ? state.inputs.velLMms : 0.0f;
    float velR = haveVel ? state.inputs.velRMms : 0.0f;
    bool haveTwist = (config.tlmFields & TLM_FIELD_TWIST) != 0;

    char modeChar = 'I';
    switch (motionController.mode()) {
        case DriveMode::STREAMING: modeChar = 'S'; break;
        case DriveMode::DISTANCE:  modeChar = 'D'; break;
        case DriveMode::GO_TO:     modeChar = 'G'; break;
        case DriveMode::VELOCITY:  modeChar = 'V'; break;
        // N13 (030-010): TIMED removed — T command runs as VELOCITY; mode=T
        // was unreachable in firmware. Host parser handles mode=T gracefully
        // for backward-compatibility with old logs.
        default:                   modeChar = 'I'; break;
    }

    int pos = 0, rem = len;
    int n = snprintf(buf + pos, (size_t)rem, "TLM t=%lu mode=%c seq=%u",
                     (unsigned long)t_sample, modeChar, (unsigned)_tlmSeq++);
    if (n > 0 && n < rem) { pos += n; rem -= n; }
    if (config.tlmFields & TLM_FIELD_ENC) {
        n = snprintf(buf + pos, (size_t)rem, " enc=%d,%d", (int)encL, (int)encR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (config.tlmFields & TLM_FIELD_POSE) {
        n = snprintf(buf + pos, (size_t)rem, " pose=%d,%d,%d",
                     (int)pose_x, (int)pose_y, (int)pose_h);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveVel) {
        n = snprintf(buf + pos, (size_t)rem, " vel=%d,%d", (int)velL, (int)velR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveTwist) {
        // fusedV is body linear speed in mm/s (integer).
        // fusedOmega is yaw rate in rad/s; convert to mrad/s (integer) matching
        // the omega_mrads convention used by VW command and NezhaProtocol.vw().
        n = snprintf(buf + pos, (size_t)rem, " twist=%d,%d",
                     (int)state.inputs.fusedV,
                     (int)(state.inputs.fusedOmega * 1000.0f));
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    // N8 (030-008): gate raw otos= on freshness — same 2×lagMs rule as
    // line/color above.  otos.valid stays true after the first success; if the
    // sensor goes dark the last-good pose would be emitted forever without
    // the freshness check.
    if ((config.tlmFields & TLM_FIELD_OTOS) &&
        state.inputs.otos.valid &&
        (t_sample - state.inputs.otos.lastUpdMs
             <= 2u * state.inputs.otos.lagMs)) {
        // Raw OTOS pose (pre-fusion): x,y mm and heading in centidegrees,
        // matching the pose= field encoding. Lets the host plot the raw OTOS
        // sensor track alongside enc-derived and fused pose. 18000/pi cdeg/rad.
        n = snprintf(buf + pos, (size_t)rem, " otos=%d,%d,%d",
                     (int)state.inputs.otosX,
                     (int)state.inputs.otosY,
                     (int)(state.inputs.otosH * 5729.5779513f));
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveLine) {
        n = snprintf(buf + pos, (size_t)rem, " line=%u,%u,%u,%u",
                     (unsigned)state.inputs.line[0], (unsigned)state.inputs.line[1],
                     (unsigned)state.inputs.line[2], (unsigned)state.inputs.line[3]);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveColor) {
        n = snprintf(buf + pos, (size_t)rem, " color=%u,%u,%u,%u",
                     (unsigned)state.inputs.colorR, (unsigned)state.inputs.colorG,
                     (unsigned)state.inputs.colorB, (unsigned)state.inputs.colorC);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (config.tlmFields & TLM_FIELD_EKFREJ) {
        // Cumulative EKF gate rejection count — all channels (pos, heading, velocity).
        // Sprint 024-005: emitted as ekf_rej=<n> for divergence visibility.
        n = snprintf(buf + pos, (size_t)rem, " ekf_rej=%d",
                     odometry.ekfRejectCount());
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    buf[pos] = '\0';
    return pos;
}

// ---------------------------------------------------------------------------
// telemetryEmit — gate and emit the periodic TLM frame.
//
// D10 idle-rate change (028-005): the stream no longer goes silent when the
// robot is stopped.  When idle, the effective period is max(tlmPeriodMs, 500)
// so the host can distinguish "robot idle" from "serial dropped."
// The clamp (tlmPeriodMs < 20 → 20) is enforced in handleStream, not here;
// telemetryEmit must NOT write to config.
// ---------------------------------------------------------------------------

void Robot::telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    if (config.tlmPeriodMs <= 0) return;

    // N3 null guard (030-003): _tlmBoundFn stays nullptr until STREAM binds the
    // channel.  SET tlmPeriod without a prior STREAM must not reach fn(…) — a
    // null fn-pointer call is a HardFault on the micro:bit.  Silent suppression
    // matches the Robot.h:164-169 comment ("nullptr means TLM is suppressed").
    if (fn == nullptr) return;

    // Idle-rate: when stopped, slow down to max(period, 500 ms) so the stream
    // stays alive but doesn't flood the link with idle noise.
    static constexpr uint32_t kIdleMinMs = 500;
    static constexpr uint32_t kGraceMs   = 400;
    if (motionController.mode() != DriveMode::IDLE) _lastActiveMs = now_ms;
    bool stopped = ((now_ms - _lastActiveMs) > kGraceMs);

    uint32_t effectivePeriod = stopped
        ? ((uint32_t)config.tlmPeriodMs > kIdleMinMs
               ? (uint32_t)config.tlmPeriodMs
               : kIdleMinMs)
        : (uint32_t)config.tlmPeriodMs;

    if ((now_ms - _lastTlmMs) < effectivePeriod) return;

    char tlmBuf[160];
    buildTlmFrame(tlmBuf, sizeof(tlmBuf));
    fn(tlmBuf, ctx);
    _lastTlmMs = now_ms;
}

// ===========================================================================
// buildCommandTable — system command handlers + aggregation
//
// All system command handlers are static functions defined here.
// handlerCtx is always RobotSysCtx* (cast inside each handler).
// ===========================================================================

// ---------------------------------------------------------------------------
// Internal accessor — cast handlerCtx to RobotSysCtx*.
// ---------------------------------------------------------------------------
namespace {

static RobotSysCtx& ctxFrom(void* p)
{
    return *reinterpret_cast<RobotSysCtx*>(p);
}

// ---------------------------------------------------------------------------
// HELLO — raw DEVICE banner (no OK wrapper).
//   prefix "HELLO"; parseFn nullptr; no args.
//   Output: DEVICE:NEZHA2:robot:<name>:<serial>
// ---------------------------------------------------------------------------

static ParseResult parseHello(const char* const* /*tokens*/, int /*ntokens*/,
                               const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleHello(const ArgList& /*args*/, const char* /*corrId*/,
                         ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    (void)handlerCtx;
    const char* name   = microbit_friendly_name();
    uint32_t    serial = microbit_serial_number();
    char banner[64];
    snprintf(banner, sizeof(banner),
             "DEVICE:NEZHA2:robot:%s:%lu", name, (unsigned long)serial);
    replyFn(banner, replyCtx);
}

// ---------------------------------------------------------------------------
// PING — clock-sync probe.
//   prefix "PING"; parseFn nullptr.
//   Reply: OK pong t=<ms>
// ---------------------------------------------------------------------------

static ParseResult parsePing(const char* const* /*tokens*/, int /*ntokens*/,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handlePing(const ArgList& /*args*/, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    uint32_t t = robot->systemTime();
    char rbuf[64];
    char body[32];
    snprintf(body, sizeof(body), "t=%lu", (unsigned long)t);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "pong", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// ECHO — echo payload tokens back.
//   prefix "ECHO"; parseFn stores tokens as STR args.
//   Reply: OK echo <joined tokens>
// ---------------------------------------------------------------------------

static ParseResult parseEcho(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    // Store each token as a STR arg; handler reassembles them.
    ParseResult r;
    r.ok = true;
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    r.args.count = n;
    for (int i = 0; i < n; ++i) {
        r.args.args[i].type = ArgType::STR;
        r.args.args[i].ival = 0;
        r.args.args[i].fval = 0.0f;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(r.args.args[i].sval) - 1; ++j)
            r.args.args[i].sval[j] = tokens[i][j];
        r.args.args[i].sval[j] = '\0';
    }
    return r;
}

static void handleEcho(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    (void)handlerCtx;
    // Reassemble tokens into a single space-joined payload.
    char payload[512];
    int pos = 0;
    for (int i = 0; i < args.count && pos < (int)sizeof(payload) - 2; ++i) {
        if (i > 0) payload[pos++] = ' ';
        for (const char* c = args.args[i].sval;
             *c != '\0' && pos < (int)sizeof(payload) - 1; ++c)
            payload[pos++] = *c;
    }
    payload[pos] = '\0';

    char rbuf[520];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "echo", payload, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// ID — full identification response.
//   prefix "ID"; parseFn nullptr.
//   Reply: ID model=Nezha2 name=<n> serial=<s> fw=<ver> proto=2 caps=<c>
// ---------------------------------------------------------------------------

static ParseResult parseId(const char* const* /*tokens*/, int /*ntokens*/,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleId(const ArgList& /*args*/, const char* corrId,
                      ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot  = ctxFrom(handlerCtx).robot;
    const char* name   = microbit_friendly_name();
    uint32_t    serial = microbit_serial_number();

    char caps[64];
    caps[0] = '\0';
    bool first = true;
    auto addCap = [&](const char* cap) {
        if (!first) {
            int n = (int)strlen(caps);
            caps[n] = ','; caps[n+1] = '\0';
        }
        int rem = (int)(sizeof(caps) - strlen(caps) - 1);
        if (rem > 0) strncat(caps, cap, (size_t)rem);
        first = false;
    };
    if (robot->otos.is_initialized())        addCap("otos");
    if (robot->line.is_initialized())        addCap("line");
    if (robot->colorSensor.is_initialized()) addCap("color");
    addCap("portio");

    char rbuf[520];
    if (corrId && corrId[0] != '\0') {
        snprintf(rbuf, sizeof(rbuf),
                 "ID model=Nezha2 name=%s serial=%lu fw=%s proto=%d caps=%s #%s",
                 name, (unsigned long)serial, FIRMWARE_VERSION, PROTO_VERSION,
                 caps, corrId);
    } else {
        snprintf(rbuf, sizeof(rbuf),
                 "ID model=Nezha2 name=%s serial=%lu fw=%s proto=%d caps=%s",
                 name, (unsigned long)serial, FIRMWARE_VERSION, PROTO_VERSION,
                 caps);
    }
    replyFn(rbuf, replyCtx);
}

// ---------------------------------------------------------------------------
// VER — firmware/protocol version query.
//   prefix "VER"; parseFn nullptr.
//   Reply: OK ver fw=<ver> proto=2
// ---------------------------------------------------------------------------

static ParseResult parseVer(const char* const* /*tokens*/, int /*ntokens*/,
                             const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleVer(const ArgList& /*args*/, const char* corrId,
                       ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/)
{
    char rbuf[64];
    char body[64];
    snprintf(body, sizeof(body), "fw=%s proto=%d", FIRMWARE_VERSION, PROTO_VERSION);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "ver", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// HELP — list all verbs.
//   prefix "HELP"; parseFn nullptr.
//   Reply: OK help <verb list>
// ---------------------------------------------------------------------------

static ParseResult parseHelp(const char* const* /*tokens*/, int /*ntokens*/,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleHelp(const ArgList& /*args*/, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/)
{
    char rbuf[520];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "help",
        "PING ECHO ID VER HELP SET GET GET VEL STREAM SNAP "
        "S T D G R TURN RT VW RF X STOP GRIP ZERO + SAFE "
        "OI OZ OR OP OV OL OA P PA "
        "[sensor=<ch>:<op>:<thr>]",
        corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// SNAP — synchronous telemetry frame.
//   prefix "SNAP"; parseFn nullptr.
//   Reply: TLM ... (raw frame, not OK-wrapped)
//
// Tick-ordering limitation (field-024 lead A, 028-001):
//   SNAP is dispatched by cmd.dequeueOne() at the START of loopTickOnce(),
//   BEFORE driveAdvance() runs.  At a mode-transition boundary (e.g. the
//   first tick after a G/T/D command arrives), SNAP reports end-of-last-tick
//   state — so mode=IDLE and enc=0 are possible even while the robot is
//   physically moving.  After the first post-command tick, SNAP reflects live
//   state correctly.
//
//   The real fix for host-visible frame staleness is D10 seq numbers
//   (ticket 028-005): the shared _tlmSeq counter on both SNAP and STREAM lets
//   the host detect/skip frames from before a motion phase started.
// ---------------------------------------------------------------------------

static ParseResult parseSnap(const char* const* /*tokens*/, int /*ntokens*/,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleSnap(const ArgList& /*args*/, const char* /*corrId*/,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    char tlmBuf[160];
    robot->buildTlmFrame(tlmBuf, sizeof(tlmBuf));
    replyFn(tlmBuf, replyCtx);
}

// ---------------------------------------------------------------------------
// ZERO — zero encoders and/or odometry.
//   prefix "ZERO"; parseFn passes "enc"/"pose" token args.
//   Reply: OK zero <enc|pose|enc pose>
// ---------------------------------------------------------------------------

static ParseResult parseZero(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    if (ntokens < 1) {
        r.ok = false;
        r.err = { "badarg", nullptr };
        return r;
    }
    // Accept enc, pose, T, D. At least one must be present.
    bool hasEnc  = false;
    bool hasPose = false;
    bool hasT    = false;
    bool hasD    = false;
    for (int i = 0; i < ntokens; ++i) {
        if (strcmp(tokens[i], "enc")  == 0) hasEnc  = true;
        if (strcmp(tokens[i], "pose") == 0) hasPose = true;
        if (strcmp(tokens[i], "T")    == 0) hasT    = true;
        if (strcmp(tokens[i], "D")    == 0) hasD    = true;
    }
    if (!hasEnc && !hasPose && !hasT && !hasD) {
        r.ok = false;
        r.err = { "badarg", nullptr };
        return r;
    }
    // Pass tokens as STR args.
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    r.ok = true;
    r.args.count = n;
    for (int i = 0; i < n; ++i) {
        r.args.args[i].type = ArgType::STR;
        r.args.args[i].ival = 0;
        r.args.args[i].fval = 0.0f;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(r.args.args[i].sval) - 1; ++j)
            r.args.args[i].sval[j] = tokens[i][j];
        r.args.args[i].sval[j] = '\0';
    }
    return r;
}

static void handleZero(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;

    bool doEnc  = false;
    bool doPose = false;
    bool doT    = false;
    bool doD    = false;
    for (int i = 0; i < args.count; ++i) {
        if (strcmp(args.args[i].sval, "enc")  == 0) doEnc  = true;
        if (strcmp(args.args[i].sval, "pose") == 0) doPose = true;
        if (strcmp(args.args[i].sval, "T")    == 0) doT    = true;
        if (strcmp(args.args[i].sval, "D")    == 0) doD    = true;
    }
    // ZERO enc — atomic encoder reset: hardware accumulators, MC velocity
    // baselines, outlier-filter baseline, and Odometry encoder snapshot.
    // (N1 fix, sprint 030-001: replaces bare resetEncoderAccumulators() which
    // left state.inputs.encLMm/R stale, freezing encoder reads for ~target mm.)
    if (doEnc)  robot->resetEncoders();
    if (doPose) robot->odometry.zero(robot->state.inputs);
    // ZERO T — set timer baseline for HaltController TIME conditions.
    if (doT) {
        robot->haltController.setTimerBaseline(robot->systemTime());
    }
    // ZERO D — set distance baseline for HaltController DISTANCE conditions.
    if (doD) {
        float enc_avg = (robot->state.inputs.encLMm + robot->state.inputs.encRMm) * 0.5f;
        robot->haltController.setDistBaseline(enc_avg);
    }

    // Build response body listing what was zeroed.
    char rbuf[64];
    char body[32];
    int  bpos = 0;
    int  brem = (int)sizeof(body);
    auto append = [&](const char* tok) {
        int n = snprintf(body + bpos, (size_t)brem, "%s%s",
                         bpos > 0 ? " " : "", tok);
        if (n > 0 && n < brem) { bpos += n; brem -= n; }
    };
    if (doEnc)  append("enc");
    if (doPose) append("pose");
    if (doT)    append("T");
    if (doD)    append("D");
    body[bpos] = '\0';
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "zero", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// STREAM — configure telemetry stream period and/or field mask.
//   prefix "STREAM"; parseFn passes period int or fields= string.
//   Reply: OK stream period=<ms> | OK stream fields=<csv>
// ---------------------------------------------------------------------------

static ParseResult parseStream(const char* const* tokens, int ntokens,
                                const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    r.ok = true;
    // Pass tokens as raw STR args.
    // "STREAM <ms>" → args[0].sval = "<ms>"  (parsed as int by handler)
    // "STREAM fields=enc,pose" → args[0].sval = "fields=enc,pose"  (handler checks prefix)
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    r.args.count = n;
    for (int i = 0; i < n; ++i) {
        r.args.args[i].type = ArgType::STR;
        r.args.args[i].ival = 0;
        r.args.args[i].fval = 0.0f;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(r.args.args[i].sval) - 1; ++j)
            r.args.args[i].sval[j] = tokens[i][j];
        r.args.args[i].sval[j] = '\0';
    }
    return r;
}

static void handleStream(const ArgList& args, const char* corrId,
                          ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    char rbuf[520];

    // Scan args for a "fields=..." entry.
    for (int i = 0; i < args.count; ++i) {
        const char* sv = args.args[i].sval;
        if (strncmp(sv, "fields=", 7) == 0) {
            const char* fp = sv + 7;
            uint8_t mask = 0;
            char fbuf[64];
            int flen = 0;
            for (const char* c = fp; ; ++c) {
                bool end = (*c == '\0' || *c == ',');
                if (!end && flen < (int)(sizeof(fbuf) - 1))
                    fbuf[flen++] = *c;
                if (end) {
                    fbuf[flen] = '\0';
                    if (strcmp(fbuf, "enc")     == 0) mask |= TLM_FIELD_ENC;
                    if (strcmp(fbuf, "pose")    == 0) mask |= TLM_FIELD_POSE;
                    if (strcmp(fbuf, "vel")     == 0) mask |= TLM_FIELD_VEL;
                    if (strcmp(fbuf, "line")    == 0) mask |= TLM_FIELD_LINE;
                    if (strcmp(fbuf, "color")   == 0) mask |= TLM_FIELD_COLOR;
                    if (strcmp(fbuf, "twist")   == 0) mask |= TLM_FIELD_TWIST;
                    if (strcmp(fbuf, "otos")    == 0) mask |= TLM_FIELD_OTOS;
                    if (strcmp(fbuf, "ekf_rej") == 0) mask |= TLM_FIELD_EKFREJ;
                    flen = 0;
                    if (*c == '\0') break;
                }
            }
            robot->config.tlmFields = mask ? mask : TLM_FIELD_ALL;

            // Reconstruct the fields string for the response body.
            char body[80];
            int bpos = 0;
            bool needComma = false;
            const struct { uint8_t bit; const char* name; } kFieldNames[] = {
                { TLM_FIELD_ENC,    "enc"     },
                { TLM_FIELD_POSE,   "pose"    },
                { TLM_FIELD_VEL,    "vel"     },
                { TLM_FIELD_LINE,   "line"    },
                { TLM_FIELD_COLOR,  "color"   },
                { TLM_FIELD_TWIST,  "twist"   },
                { TLM_FIELD_OTOS,   "otos"    },
                { TLM_FIELD_EKFREJ, "ekf_rej" },
            };
            int brem = (int)sizeof(body);
            int bw = snprintf(body + bpos, (size_t)brem, "fields=");
            if (bw > 0 && bw < brem) { bpos += bw; brem -= bw; }
            for (int fi = 0; fi < 8 && brem > 1; ++fi) {
                if (robot->config.tlmFields & kFieldNames[fi].bit) {
                    if (needComma) { body[bpos++] = ','; --brem; }
                    bw = snprintf(body + bpos, (size_t)brem, "%s", kFieldNames[fi].name);
                    if (bw > 0 && bw < brem) { bpos += bw; brem -= bw; }
                    needComma = true;
                }
            }
            body[bpos] = '\0';
            CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stream", body,
                                      corrId, replyFn, replyCtx);
            return;
        }
    }

    // No fields= — expect a positional period arg.
    if (args.count < 1) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "usage: STREAM <ms>",
                                   corrId, replyFn, replyCtx);
        return;
    }
    int32_t ms = (int32_t)atoi(args.args[0].sval);
    if (ms < 0) ms = 0;
    if (ms > 0 && ms < 20) ms = 20;  // clamp to 50 Hz max (D10 028-005: enforced here, NOT in telemetryEmit)
    robot->config.tlmPeriodMs = ms;

    // D10 channel binding (028-005): bind the TLM stream to the channel that
    // issued this STREAM command.  runCommsIn uses _tlmBoundCtx to identify
    // the channel and derive the TLM-appropriate reply fn (serialReplyTlm
    // for serial, radioReply for radio).  Commands on other channels do not
    // redirect the stream.
    //
    // N3 fix (030-003): also store the caller's replyFn as _tlmBoundFn so that
    // telemetryEmit (now using _tlmBoundFn/_tlmBoundCtx directly) has a valid fn
    // in both the sim path (replyFn = storeReply) and firmware path.  In firmware,
    // runCommsIn overwrites _tlmBoundFn on the next iteration with the correct
    // channel fn (serialReplyTlm or radioReply derived from _tlmBoundCtx), so
    // _tlmBoundFn is always the pair that matches _tlmBoundCtx.
    robot->_tlmBoundFn  = replyFn;
    robot->_tlmBoundCtx = replyCtx;

    char body[32];
    snprintf(body, sizeof(body), "period=%d", (int)ms);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stream", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// RF — radio channel get/set.
//   prefix "RF"; parseFn passes optional channel as INT arg.
//   Reply: OK rf chan=<n> group=10
// ---------------------------------------------------------------------------

static ParseResult parseRf(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    r.ok = true;
    if (ntokens >= 1) {
        r.args.count = 1;
        r.args.args[0].type = ArgType::INT;
        r.args.args[0].ival = atoi(tokens[0]);
        r.args.args[0].fval = 0.0f;
        r.args.args[0].sval[0] = '\0';
    } else {
        r.args.count = 0;
    }
    return r;
}

static void handleRf(const ArgList& args, const char* corrId,
                      ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    LoopScheduler* sched = ctxFrom(handlerCtx).sched;
    char rbuf[64];
    if (sched == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noradio", nullptr,
                                   corrId, replyFn, replyCtx);
        return;
    }
#ifndef HOST_BUILD
    Radio& radio = sched->comm().radio();

    if (args.count < 1) {
        // Query.
        char body[32];
        snprintf(body, sizeof(body), "chan=%d group=%d",
                 radio.channel(), radiochan::kGroup);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rf", body,
                                  corrId, replyFn, replyCtx);
        return;
    }

    int ch = args.args[0].ival;
    if (ch < radiochan::kMin || ch > radiochan::kMax) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "range", "chan",
                                   corrId, replyFn, replyCtx);
        return;
    }
    // Persist first, then reply on the OLD channel, then re-tune.
    radiochan::save(sched->uBit().storage, ch);
    char body[32];
    snprintf(body, sizeof(body), "chan=%d group=%d", ch, radiochan::kGroup);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rf", body,
                              corrId, replyFn, replyCtx);
    radio.setChannel(ch);
#else
    (void)args;
#endif
}

// ---------------------------------------------------------------------------
// GET VEL — per-wheel velocity readout (separate descriptor from GET).
//   prefix "GET VEL"; parseFn nullptr.
//   Reply: OK get vel=<vL>:E,<vR>:E
// ---------------------------------------------------------------------------

static ParseResult parseGetVel(const char* const* /*tokens*/, int /*ntokens*/,
                                const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleGetVel(const ArgList& /*args*/, const char* corrId,
                          ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    float vL = robot->state.inputs.velLMms;
    float vR = robot->state.inputs.velRMms;
    char rbuf[64];
    char body[48];
    snprintf(body, sizeof(body), "vel=%d:E,%d:E", (int)vL, (int)vR);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "get", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// parseGet — convert positional key-name tokens into STR args for handleGet.
//   Each token becomes args[i].sval = key name.
// ---------------------------------------------------------------------------

static ParseResult parseGet(const char* const* tokens, int ntokens,
                             const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    r.ok = true;
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    r.args.count = n;
    for (int i = 0; i < n; ++i) {
        r.args.args[i].type = ArgType::STR;
        r.args.args[i].ival = 0;
        r.args.args[i].fval = 0.0f;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(r.args.args[i].sval) - 1; ++j)
            r.args.args[i].sval[j] = tokens[i][j];
        r.args.args[i].sval[j] = '\0';
    }
    return r;
}

// ---------------------------------------------------------------------------
// parseSet — convert kv pairs into "key=value" STR args for handleSet.
// ---------------------------------------------------------------------------

static ParseResult parseSet(const char* const* /*tokens*/, int /*ntokens*/,
                             const KVPair* kvs, int nkv)
{
    ParseResult r;
    if (nkv == 0) {
        r.ok = false;
        r.err = { "badarg", "no key=value pairs" };
        return r;
    }
    r.ok = true;
    int n = (nkv > MAX_ARGS) ? MAX_ARGS : nkv;
    r.args.count = 0;
    for (int i = 0; i < n; ++i) {
        if (!kvs[i].key) continue;
        char* dst = r.args.args[r.args.count].sval;
        int cap = (int)(sizeof(r.args.args[0].sval) - 1);
        int written = snprintf(dst, (size_t)(cap + 1), "%s=%s",
                               kvs[i].key, kvs[i].value);
        if (written > cap) dst[cap] = '\0';
        r.args.args[r.args.count].type = ArgType::STR;
        r.args.args[r.args.count].ival = 0;
        r.args.args[r.args.count].fval = 0.0f;
        ++r.args.count;
    }
    return r;
}

// ---------------------------------------------------------------------------
// + — keepalive command.
//   prefix "+"; parseFn nullptr (no args).
//   Resets the system watchdog timestamp.
//   Reply: OK keepalive
// ---------------------------------------------------------------------------

static ParseResult parseKeepalive(const char* const* /*tokens*/, int /*ntokens*/,
                                   const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleKeepalive(const ArgList& /*args*/, const char* /*corrId*/,
                              ReplyFn /*replyFn*/, void* /*replyCtx*/, void* handlerCtx)
{
    // Quiet keepalive (sprint 024-003): suppress the "OK keepalive" reply.
    // At 6.7 Hz the acks competed with TLM frames for the 250-byte TX buffer;
    // the host already filters them.  The watchdog reset (firmware side) and
    // the sim watchdog arm (sim_api.cpp via sim_command) are the only effects.
#ifndef HOST_BUILD
    LoopScheduler* sched = ctxFrom(handlerCtx).sched;
    Robot*         robot = ctxFrom(handlerCtx).robot;
    if (sched != nullptr) {
        sched->resetWatchdog(robot->systemTime());
    }
#else
    (void)handlerCtx;
#endif
    // No reply emitted (quiet keepalive).
}

// ---------------------------------------------------------------------------
// SAFE — enable/disable the system safety-stop watchdog and set its timeout.
//   SAFE                 → query: "OK safety on|off timeout=<ms>"
//   SAFE off  (or SAFE 0)→ disable the watchdog (no keepalives required)
//   SAFE on   [<ms>]     → enable; optional <ms> sets sTimeoutMs
//   SAFE <ms>            → <ms> > 0: enable + set timeout; 0: disable
// Tokens are passed through as STR args (same as parseGet).
// ---------------------------------------------------------------------------

static ParseResult parseSafe(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    r.ok = true;
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    r.args.count = n;
    for (int i = 0; i < n; ++i) {
        r.args.args[i].type = ArgType::STR;
        r.args.args[i].ival = 0;
        r.args.args[i].fval = 0.0f;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(r.args.args[i].sval) - 1; ++j)
            r.args.args[i].sval[j] = tokens[i][j];
        r.args.args[i].sval[j] = '\0';
    }
    return r;
}

static void handleSafe(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    char rbuf[80];
    Robot* robot = ctxFrom(handlerCtx).robot;
    if (robot == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noctx", "SAFE",
                                   corrId, replyFn, replyCtx);
        return;
    }
    RobotConfig& cfg = robot->config;

    if (args.count >= 1) {
        const char* a0 = args.args[0].sval;
        if (strcmp(a0, "off") == 0) {
            // One-shot disable: do NOT clear safetyEnabled directly.
            // Instead arm the one-shot flag in MotionController so safety
            // is automatically restored when the next motion command begins.
            // This prevents SAFE off from becoming a permanent foot-gun.
            robot->motionController.disableSafetyOneShot();
            // Reflect the transient "off" state in the reply (safetyEnabled
            // will be re-armed by MotionController on the next begin*() call,
            // but for the duration of any current-or-next command the watchdog
            // is suppressed via _safeOneShotDisable).
            cfg.safetyEnabled = false;
        } else if (strcmp(a0, "on") == 0) {
            cfg.safetyEnabled = true;
            if (args.count >= 2) {
                int ms = atoi(args.args[1].sval);
                if (ms > 0) cfg.sTimeoutMs = ms;
            }
        } else {
            // Numeric form: SAFE <ms>  (0 → off, >0 → on with that timeout).
            int ms = atoi(a0);
            if (ms <= 0) {
                // Same one-shot treatment as "SAFE off".
                robot->motionController.disableSafetyOneShot();
                cfg.safetyEnabled = false;
            } else {
                cfg.safetyEnabled = true;
                cfg.sTimeoutMs = ms;
            }
        }
    }

    char body[48];
    snprintf(body, sizeof(body), "%s timeout=%d",
             cfg.safetyEnabled ? "on" : "off", (int)cfg.sTimeoutMs);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "safety", body,
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// SI — set the odometry world pose directly (what G reads via getPoseFloat).
//   SI <x_mm> <y_mm> <h_cdeg>
// Establishes the robot's onboard pose from an external fix (e.g. the camera)
// so a subsequent G/D/TURN drives in the correct world frame. This is the pose
// the motion controller reads — unlike OV, which only nudges the raw OTOS chip.
// Reply: OK setpose x=<mm> y=<mm> h=<cdeg>
// ---------------------------------------------------------------------------

static ParseResult parseSI(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    if (ntokens < 3) {
        r.ok = false;
        r.err = { "badarg", "SI x_mm y_mm h_cdeg" };
        return r;
    }
    r.ok = true;
    r.args.count = 3;
    r.args.args[0].type = ArgType::INT; r.args.args[0].ival = atoi(tokens[0]);
    r.args.args[1].type = ArgType::INT; r.args.args[1].ival = atoi(tokens[1]);
    r.args.args[2].type = ArgType::INT; r.args.args[2].ival = atoi(tokens[2]);
    return r;
}

static void handleSI(const ArgList& args, const char* corrId,
                      ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    char rbuf[80];
    Robot* robot = ctxFrom(handlerCtx).robot;
    if (robot == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noctx", "SI",
                                   corrId, replyFn, replyCtx);
        return;
    }
    int32_t x_mm   = args.args[0].ival;
    int32_t y_mm   = args.args[1].ival;
    int32_t h_cdeg = args.args[2].ival;
    robot->odometry.setPose(robot->state.inputs, x_mm, y_mm, h_cdeg);
    char body[48];
    snprintf(body, sizeof(body), "x=%d y=%d h=%d", (int)x_mm, (int)y_mm, (int)h_cdeg);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "setpose", body,
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// HALT — user-facing named stop-condition commands.
//
// Wire formats:
//   HALT TIME <ms>          → OK HALT id=<n>
//   HALT TIME <ms> SOFT     → OK HALT id=<n>
//   HALT DIST <mm>          → OK HALT id=<n>
//   HALT DIST <mm> SOFT     → OK HALT id=<n>
//   HALT LINE ANY <GE|LE> <threshold>       → OK HALT id=<n>
//   HALT LINE ANY <GE|LE> <threshold> SOFT  → OK HALT id=<n>
//   HALT CLEAR              → OK HALT cleared=<count>
//   HALT LIST               → one "OK HALT id=<n> str=..." line per entry + OK HALT list
//
// parseFn: passes tokens as STR args (first arg is the sub-verb: TIME, DIST,
// LINE, CLEAR, LIST). Handler dispatches on args[0].sval.
// ---------------------------------------------------------------------------

static ParseResult parseHalt(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    if (ntokens < 1) {
        r.ok = false;
        r.err = { "badarg", "usage: HALT TIME|DIST|POS|COLOR|LINE|CLEAR|INFO|LIST ..." };
        return r;
    }
    // Validate sub-verb.
    const char* sv = tokens[0];
    if (strcmp(sv, "TIME")  != 0 && strcmp(sv, "DIST")  != 0 &&
        strcmp(sv, "LINE")  != 0 && strcmp(sv, "CLEAR") != 0 &&
        strcmp(sv, "LIST")  != 0 && strcmp(sv, "POS")   != 0 &&
        strcmp(sv, "COLOR") != 0 && strcmp(sv, "INFO")  != 0) {
        r.ok = false;
        r.err = { "badarg", "usage: HALT TIME|DIST|POS|COLOR|LINE|CLEAR|INFO|LIST ..." };
        return r;
    }
    // Pass all tokens as STR args.
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    r.ok = true;
    r.args.count = n;
    for (int i = 0; i < n; ++i) {
        r.args.args[i].type = ArgType::STR;
        r.args.args[i].ival = 0;
        r.args.args[i].fval = 0.0f;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(r.args.args[i].sval) - 1; ++j)
            r.args.args[i].sval[j] = tokens[i][j];
        r.args.args[i].sval[j] = '\0';
    }
    return r;
}

static void handleHalt(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    char rbuf[128];

    if (args.count < 1) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                   "usage: HALT TIME|DIST|LINE|CLEAR|LIST ...",
                                   corrId, replyFn, replyCtx);
        return;
    }

    const char* sv = args.args[0].sval;

    // ---- CLEAR ----
    if (strcmp(sv, "CLEAR") == 0) {
        if (args.count >= 2) {
            // HALT CLEAR <id> — remove one entry by id.
            uint8_t rmid = (uint8_t)atoi(args.args[1].sval);
            bool removed = robot->haltController.remove(rmid);
            if (!removed) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "notfound", "id",
                                           corrId, replyFn, replyCtx);
                return;
            }
            char body[32];
            snprintf(body, sizeof(body), "cleared id=%u", (unsigned)rmid);
            CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                       corrId, replyFn, replyCtx);
        } else {
            // HALT CLEAR — remove all entries.
            int n = robot->haltController.clear();
            char body[32];
            snprintf(body, sizeof(body), "cleared=%d", n);
            CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                       corrId, replyFn, replyCtx);
        }
        return;
    }

    // ---- LIST ----
    if (strcmp(sv, "LIST") == 0) {
        robot->haltController.list(replyFn, replyCtx);
        char body[32];
        snprintf(body, sizeof(body), "list count=%d",
                 robot->haltController.count());
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // ---- TIME ----
    if (strcmp(sv, "TIME") == 0) {
        if (args.count < 2) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "usage: HALT TIME <ms> [SOFT]",
                                       corrId, replyFn, replyCtx);
            return;
        }
        float ms = (float)atof(args.args[1].sval);
        StopStyle style = StopStyle::HARD;
        if (args.count >= 3 && strcmp(args.args[2].sval, "SOFT") == 0)
            style = StopStyle::SOFT;

        StopCondition cond = makeTimeStop(ms);
        // Build a label string for HALT LIST.
        char label[40];
        snprintf(label, sizeof(label), "TIME %g%s", ms,
                 style == StopStyle::SOFT ? " SOFT" : "");
        // Capture registration-time baseline so the condition fires ~ms after
        // now, not ~ms after boot (N10 fix).
        uint32_t now_ms   = robot->systemTime();
        float    enc_avg  = (robot->state.inputs.encLMm + robot->state.inputs.encRMm) * 0.5f;
        int id = robot->haltController.add(cond, style, label, now_ms, enc_avg);
        if (id < 0) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full",
                                       "halt table full (max 8)",
                                       corrId, replyFn, replyCtx);
            return;
        }
        char body[32];
        snprintf(body, sizeof(body), "id=%d", id);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // ---- DIST ----
    if (strcmp(sv, "DIST") == 0) {
        if (args.count < 2) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "usage: HALT DIST <mm> [SOFT]",
                                       corrId, replyFn, replyCtx);
            return;
        }
        float mm = (float)atof(args.args[1].sval);
        StopStyle style = StopStyle::HARD;
        if (args.count >= 3 && strcmp(args.args[2].sval, "SOFT") == 0)
            style = StopStyle::SOFT;

        StopCondition cond = makeDistanceStop(mm);
        char label[40];
        snprintf(label, sizeof(label), "DIST %g%s", mm,
                 style == StopStyle::SOFT ? " SOFT" : "");
        // Capture registration-time baseline so the condition fires ~mm after
        // the current encoder position, not from boot (N10 fix).
        uint32_t now_ms_d  = robot->systemTime();
        float    enc_avg_d = (robot->state.inputs.encLMm + robot->state.inputs.encRMm) * 0.5f;
        int id = robot->haltController.add(cond, style, label, now_ms_d, enc_avg_d);
        if (id < 0) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full",
                                       "halt table full (max 8)",
                                       corrId, replyFn, replyCtx);
            return;
        }
        char body[32];
        snprintf(body, sizeof(body), "id=%d", id);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // ---- LINE ANY ----
    // Wire: HALT LINE ANY <GE|LE> <threshold> [SOFT]
    if (strcmp(sv, "LINE") == 0) {
        // args: [0]=LINE [1]=ANY [2]=GE|LE [3]=threshold [4]=SOFT?
        if (args.count < 4 ||
            strcmp(args.args[1].sval, "ANY") != 0) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "usage: HALT LINE ANY GE|LE <threshold> [SOFT]",
                                       corrId, replyFn, replyCtx);
            return;
        }
        const char* opStr = args.args[2].sval;
        StopCondition::Cmp op;
        if (strcmp(opStr, "GE") == 0) {
            op = StopCondition::Cmp::GE;
        } else if (strcmp(opStr, "LE") == 0) {
            op = StopCondition::Cmp::LE;
        } else {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "op must be GE or LE",
                                       corrId, replyFn, replyCtx);
            return;
        }
        float threshold = (float)atof(args.args[3].sval);
        StopStyle style = StopStyle::HARD;
        if (args.count >= 5 && strcmp(args.args[4].sval, "SOFT") == 0)
            style = StopStyle::SOFT;

        StopCondition cond = makeLineAnyStop(threshold, op);
        // Build label: "LINE ANY GE <thr>" or "LINE ANY LE <thr> SOFT".
        // Use fixed 2-char op abbreviation and integer threshold to keep
        // label within StopEntry.str[40] and silence -Wformat-truncation.
        char label[40];
        {
            const char* opAbbrev = (op == StopCondition::Cmp::GE) ? "GE" : "LE";
            const char* softSfx  = (style == StopStyle::SOFT) ? " SOFT" : "";
            // "LINE ANY GE 65535 SOFT" = 22 chars — fits comfortably.
            snprintf(label, sizeof(label), "LINE ANY %.2s %d%s",
                     opAbbrev, (int)threshold, softSfx);
        }
        int id = robot->haltController.add(cond, style, label,
                                            robot->systemTime(),
                                            (robot->state.inputs.encLMm + robot->state.inputs.encRMm) * 0.5f);
        if (id < 0) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full",
                                       "halt table full (max 8)",
                                       corrId, replyFn, replyCtx);
            return;
        }
        char body[32];
        snprintf(body, sizeof(body), "id=%d", id);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // ---- POS ----
    if (strcmp(sv, "POS") == 0) {
        // Wire: HALT POS <x_mm> <y_mm> <radius_mm>
        if (args.count < 4) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "usage: HALT POS <x_mm> <y_mm> <radius_mm>",
                                       corrId, replyFn, replyCtx);
            return;
        }
        float x   = (float)atof(args.args[1].sval);
        float y   = (float)atof(args.args[2].sval);
        float rad = (float)atof(args.args[3].sval);
        StopStyle style = StopStyle::HARD;
        if (args.count >= 5 && strcmp(args.args[4].sval, "SOFT") == 0)
            style = StopStyle::SOFT;

        StopCondition cond = makePositionStop(x, y, rad);
        char label[40];
        // Use integer mm to keep label well within StopEntry.str[40].
        // "POS -32000 -32000 32000" = 22 chars — fits comfortably.
        snprintf(label, sizeof(label), "POS %d %d %d",
                 (int)x, (int)y, (int)rad);
        int id = robot->haltController.add(cond, style, label,
                                            robot->systemTime(),
                                            (robot->state.inputs.encLMm + robot->state.inputs.encRMm) * 0.5f);
        if (id < 0) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full",
                                       "halt table full (max 8)",
                                       corrId, replyFn, replyCtx);
            return;
        }
        char body[32];
        snprintf(body, sizeof(body), "id=%d", id);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // ---- COLOR ----
    if (strcmp(sv, "COLOR") == 0) {
        // Wire: HALT COLOR <h> <s> <v> <dist>
        if (args.count < 5) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "usage: HALT COLOR <h> <s> <v> <dist>",
                                       corrId, replyFn, replyCtx);
            return;
        }
        float h    = (float)atof(args.args[1].sval);
        float s    = (float)atof(args.args[2].sval);
        float v    = (float)atof(args.args[3].sval);
        float dist = (float)atof(args.args[4].sval);
        StopStyle style = StopStyle::HARD;
        if (args.count >= 6 && strcmp(args.args[5].sval, "SOFT") == 0)
            style = StopStyle::SOFT;

        StopCondition cond = makeColorStop(h, s, v, dist);
        char label[40];
        // Format as fixed 2-decimal for HSV floats; keep within StopEntry.str[40].
        // "COLOR 360.00 1.00 1.00 1.00" = 28 chars — fits comfortably.
        snprintf(label, sizeof(label), "COLOR %.2f %.2f %.2f %.2f",
                 (double)h, (double)s, (double)v, (double)dist);
        int id = robot->haltController.add(cond, style, label,
                                            robot->systemTime(),
                                            (robot->state.inputs.encLMm + robot->state.inputs.encRMm) * 0.5f);
        if (id < 0) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full",
                                       "halt table full (max 8)",
                                       corrId, replyFn, replyCtx);
            return;
        }
        char body[32];
        snprintf(body, sizeof(body), "id=%d", id);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // ---- INFO ----
    if (strcmp(sv, "INFO") == 0) {
        // Wire: HALT INFO <id>
        if (args.count < 2) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "usage: HALT INFO <id>",
                                       corrId, replyFn, replyCtx);
            return;
        }
        uint8_t qid = (uint8_t)atoi(args.args[1].sval);
        char infoBuf[80];
        if (!robot->haltController.info(qid, infoBuf, sizeof(infoBuf))) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "notfound", "id",
                                       corrId, replyFn, replyCtx);
            return;
        }
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", infoBuf,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // Unknown sub-verb (should not reach here after parseHalt validation).
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                               "usage: HALT TIME|DIST|POS|COLOR|LINE|CLEAR|INFO|LIST ...",
                               corrId, replyFn, replyCtx);
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Robot::buildCommandTable — aggregate all Commandables + system commands.
// ---------------------------------------------------------------------------

std::vector<CommandDescriptor> Robot::buildCommandTable(
    DebugCommandable* dbg, LoopScheduler* sched) const
{
    // Populate stable context structs (members, so pointers are valid for the
    // lifetime of this Robot).
    _cfgCtx       = { const_cast<RobotConfig*>(&config),
                      const_cast<MotorController*>(&motorController) };
    _sysCtx.robot = const_cast<Robot*>(this);
    _sysCtx.sched = sched;
    // Initialise _motionCtx for this build (sprint 026-002).
    // mc and robot pointers are already set in the constructor; vwDesc is
    // initialised by getMotionCommands() below.
    _motionCtx.mc    = const_cast<MotionController*>(&motionController);
    _motionCtx.robot = const_cast<Robot*>(this);
    // queue is set by setMotionQueue() from LoopScheduler; preserve it here.

    void* sysCtxPtr = &_sysCtx;

    std::vector<CommandDescriptor> cmds;

    // ---- Commandable members ----
    auto append = [&](std::vector<CommandDescriptor> v) {
        cmds.insert(cmds.end(), v.begin(), v.end());
    };
    // Sprint 026-002: replaced motionController.getCommands() with getMotionCommands().
    append(getMotionCommands(&_motionCtx));
    append(odometry.getCommands());
    append(portController.getCommands());
    append(servoController.getCommands());
    if (dbg) append(dbg->getCommands());

    // ---- System commands ----
    // GET VEL before GET so the longer prefix wins the linear scan.
    cmds.push_back(makeCmd("HELLO",     parseHello,     handleHello,     sysCtxPtr, "badarg")); // identify firmware + version
    cmds.push_back(makeCmd("PING",     parsePing,      handlePing,      sysCtxPtr, "badarg")); // liveness check
    cmds.push_back(makeCmd("ECHO",     parseEcho,      handleEcho,      sysCtxPtr, "badarg")); // echo tokens back
    cmds.push_back(makeCmd("ID",       parseId,        handleId,        sysCtxPtr, "badarg")); // report robot identity string
    cmds.push_back(makeCmd("VER",      parseVer,       handleVer,       sysCtxPtr, "badarg")); // report firmware version
    cmds.push_back(makeCmd("HELP",     parseHelp,      handleHelp,      sysCtxPtr, "badarg")); // list available commands
    cmds.push_back(makeCmd("SNAP",     parseSnap,      handleSnap,      sysCtxPtr, "badarg")); // emit one TLM frame on demand
    cmds.push_back(makeCmd("ZERO",     parseZero,      handleZero,      sysCtxPtr, "badarg")); // zero encoders/pose/halt-baselines
    cmds.push_back(makeCmd("HALT",     parseHalt,      handleHalt,      sysCtxPtr, "badarg")); // named stop-condition registry
    cmds.push_back(makeCmd("STREAM",   parseStream,    handleStream,    sysCtxPtr, "badarg")); // start/stop periodic TLM stream
    cmds.push_back(makeCmd("RF",       parseRf,        handleRf,        sysCtxPtr, "badarg")); // set radio channel
    cmds.push_back(makeCmd("+",        parseKeepalive, handleKeepalive, sysCtxPtr, "badarg")); // keepalive: reset watchdog
    cmds.push_back(makeCmd("SAFE",     parseSafe,      handleSafe,      sysCtxPtr, "badarg")); // enable/disable safety watchdog + set timeout
    cmds.push_back(makeCmd("SI",       parseSI,        handleSI,        sysCtxPtr, "badarg")); // set odometry world pose (x_mm y_mm h_cdeg)
    cmds.push_back(makeCmd("GET VEL",  parseGetVel,    handleGetVel,    sysCtxPtr, "badarg")); // get velocity PID params
    cmds.push_back(makeCmd("GET",      parseGet,       handleGet,       &_cfgCtx,  "badkey")); // get config value by key
    cmds.push_back(makeCmd("SET",      parseSet,       handleSet,       &_cfgCtx,  "badkey")); // set config value by key

    return cmds;
}
