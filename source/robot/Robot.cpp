#include "Robot.h"
#include "MotionController.h"
#ifndef HOST_BUILD
#include "MicroBit.h"
#include "MicroBitDevice.h"
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

static uint32_t system_timer_current_time() { return g_sim_now_ms; }
#endif

// Note: microbit_friendly_name() and microbit_serial_number() stubs
// moved to SystemCommands.cpp (split 035 A3) — only needed by system
// command handlers there.

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
                // (033-005b) Outlier rejection: increment the consecutive-reject
                // streak counter.  Saturate at 255 to avoid uint8 wrap.
                if (_filterRejectStreakR < 255) ++_filterRejectStreakR;
            } else {
                _filterRejectStreakR = 0;
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
                // (033-005b) Outlier rejection: increment streak counter.
                if (_filterRejectStreakL < 255) ++_filterRejectStreakL;
            } else {
                _filterRejectStreakL = 0;
            }
            state.inputs.encLMm = newL;
        }

        // (033-005b) Emit EVT enc_filter_hold at threshold crossing (onset only).
        // We emit exactly once when streak == threshold, not on every tick above,
        // to avoid flooding the link with repeated EVTs for a persistent hold.
        // Use _tlmBoundFn so the EVT goes to the same channel as TLM; silently
        // drop when no channel is bound (no STREAM issued yet).
        if (_filterRejectStreakR == kFilterRejectStreakThreshold &&
                _tlmBoundFn != nullptr) {
            char evtBuf[64];
            snprintf(evtBuf, sizeof(evtBuf),
                     "EVT enc_filter_hold wheel=R streak=%u",
                     (unsigned)_filterRejectStreakR);
            _tlmBoundFn(evtBuf, _tlmBoundCtx);
        }
        if (_filterRejectStreakL == kFilterRejectStreakThreshold &&
                _tlmBoundFn != nullptr) {
            char evtBuf[64];
            snprintf(evtBuf, sizeof(evtBuf),
                     "EVT enc_filter_hold wheel=L streak=%u",
                     (unsigned)_filterRejectStreakL);
            _tlmBoundFn(evtBuf, _tlmBoundCtx);
        }
    } else {
        // Not driving: reset streak counters so they don't carry over into the
        // next drive episode.
        _filterRejectStreakL = 0;
        _filterRejectStreakR = 0;
    }
    _prevDriving = driving;
    _lastControlMs = now_ms;
    // refreshedWheel=3: both wheels updated; 0: idle, no velocity update.
    motorController.controlTick(state.inputs, state.commands, now_ms, driving ? 3 : 0);

    // (033-005e) Push wedge state into Odometry after every control tick.
    // wheelWedgedL/R() return the EVT-latch state from the detector above.
    //
    // setWedgeActive: unconditionally mirrors the combined wedge flag — dTheta
    // suppression is purely Robot-owned (no external setter in tests).
    //
    // setEncOmegaHealthy: only called when a wedge is ACTIVE.  When no wedge is
    // active we do NOT call setEncOmegaHealthy(true) — this preserves any manual
    // override (e.g. sim_set_enc_omega_healthy(false) in 033-003 tests) and avoids
    // overwriting the gate each tick when everything is healthy.  The gate is only
    // restored to true when the wedge clears (anyWedged transitions false→true→false).
    bool anyWedged = motorController.wheelWedgedL() || motorController.wheelWedgedR();
    odometry.setWedgeActive(anyWedged);
    if (anyWedged) {
        // Wheel is wedged: suppress both dTheta and the omega observation.
        odometry.setEncOmegaHealthy(false);
    } else if (_prevAnyWedged) {
        // Wedge just cleared: restore omega health (encoder re-armed → moving again).
        odometry.setEncOmegaHealthy(true);
    }
    _prevAnyWedged = anyWedged;
}

// ---------------------------------------------------------------------------
// otosCorrect — EKF Kalman update from OTOS position and velocity (sprint 023).
//
// Reads OTOS position, velocity, and acceleration.  Passes position + velocity
// to correctEKF() for EKF fusion.  Stores acceleration in HardwareState for
// host telemetry via RobotState.
//
// Encoder-derived velocity is NOT fused here: as of 033-003 it is fused
// unconditionally in Odometry::predict() every tick, so fusedV/fusedOmega stay
// live even when this OTOS-gated path is skipped (lifted stand, dropout).
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

    // Encoder-derived velocity is fused unconditionally in Odometry::predict()
    // every tick (033-003), so correctEKF() fuses only the OTOS observations.
    odometry.correctEKF(state.inputs, p.x, p.y,
                        p.h,
                        vel.v_mmps, vel.omega_rads);
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

// buildTlmFrame, telemetryEmit → moved to RobotTelemetry.cpp (split 035 A3)
// buildCommandTable + all system command handlers → moved to SystemCommands.cpp (split 035 A3)
