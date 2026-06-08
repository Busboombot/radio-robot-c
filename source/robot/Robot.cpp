#include "Robot.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "DriveController.h"
#include "MicroBit.h"
#include <cstdio>
#include <cmath>

Robot::Robot(Motor&        motorL,
             Motor&        motorR,
             OtosSensor&   otos,
             LineSensor&   line,
             ColorSensor&  color,
             Servo&        gripper,
             PortIO&       portio,
             Communicator& comm,
             const RobotConfig& cfg)
    : _currentGripperAngle(0),
      _config(cfg),
      _lastTlmMs(0),
      _lastActiveMs(0),
      _motorL(motorL),
      _motorR(motorR),
      _otos(otos),
      _line(line),
      _color(color),
      _servo(gripper),
      _gripperPresent(false),
      _portio(portio),
      _comm(comm),
      _mc(_motorL, _motorR, _config),
      _odo(),
      _dc(_mc, _odo, _config),
      _state(defaultInputs(_config)),
      _lastControlMs(0),
      _lastOtosMs(0)
{
    // Devices are fully initialised before this constructor runs (main.cpp calls
    // uBit.init() and device begin() before constructing Robot).

    _gripperPresent = true;  // servo always available on P1

    // Bind authoritative HardwareState into DriveController so getPoseFloat()
    // can read pose fields (Odometry::getPose reads the struct).
    _dc.setHardwareState(&_state.inputs);

    // Bind the authoritative MotorCommands so MotorController::setTarget() /
    // startDrive() / stop() write tgtLMms/R directly (014-007).
    _mc.setCommandsRef(&_state.commands);

    // Unified TLM frame assembled in Robot::tick() — Sprint 009 ticket 005.
    // Streaming period controlled by RobotConfig::tlmPeriodMs.
}

// ---------------------------------------------------------------------------
// systemTime — robot system time in milliseconds since boot.
//
// Uses the CODAL free function system_timer_current_time() (ms resolution),
// declared in codal-core/inc/driver-models/Timer.h, available via MicroBit.h.
// ---------------------------------------------------------------------------

uint32_t Robot::systemTime() const
{
    return (uint32_t)system_timer_current_time();
}

// ---------------------------------------------------------------------------
// Drive action methods — delegate to DriveController
// ---------------------------------------------------------------------------

void Robot::stop()
{
    uint32_t now_ms = systemTime();
    // stop() with no reply fn: use a no-op sink
    _dc.stop(now_ms, [](const char*, void*){}, nullptr);
}

void Robot::streamDrive(int32_t leftMms, int32_t rightMms, ReplyFn fn, void* ctx)
{
    _dc.beginStream((float)leftMms, (float)rightMms, systemTime(),
                    _state.target, fn, ctx);
}

void Robot::velocityDrive(float v_mms, float omega_rads, ReplyFn fn, void* ctx,
                           const char* corr_id)
{
    _dc.beginVelocity(v_mms, omega_rads, systemTime(),
                      _state.target, fn, ctx, corr_id);
}

void Robot::timedDrive(int32_t leftMms, int32_t rightMms, uint32_t durationMs,
                       ReplyFn fn, void* ctx, const char* corr_id)
{
    _dc.beginTimed((float)leftMms, (float)rightMms, durationMs, systemTime(),
                   _state.target, fn, ctx, corr_id);
}

void Robot::distanceDrive(int32_t leftMms, int32_t rightMms, int32_t targetMm,
                          ReplyFn fn, void* ctx, const char* corr_id)
{
    _dc.beginDistance((float)leftMms, (float)rightMms, targetMm, systemTime(),
                      _state.target, fn, ctx, corr_id);
    // beginDistance reset the encoder accumulator to 0. The control loop's outlier
    // filter compares each new read to the stored encLMm/encRMm — so unless we
    // also zero those, the ~target→0 reset looks like a giant backward outlier and
    // gets REJECTED, freezing encLMm at the previous drive's value for the whole
    // drive. That stale value then (a) corrupts telemetry and (b) feeds the
    // velocity loop ~0 velocity → it over-drives → spasm. Reset the filter
    // baseline here so post-reset reads track cleanly. (Distance itself uses a
    // fresh getEncoderPositions read, so this only fixes the filtered cache.)
    _state.inputs.encLMm = 0.0f;
    _state.inputs.encRMm = 0.0f;
}

void Robot::goTo(float tx, float ty, float speedMms, ReplyFn fn, void* ctx,
                 const char* corr_id)
{
    _dc.beginGoTo(tx, ty, speedMms, systemTime(),
                  _state.target, fn, ctx, corr_id);
}

// ---------------------------------------------------------------------------
// Non-drive action methods
// ---------------------------------------------------------------------------

void Robot::setGripperAngle(int32_t deg)
{
    if (_gripperPresent) {
        uint8_t clamped = (deg < 0) ? 0 : (deg > 180) ? 180 : (uint8_t)deg;
        _servo.setAngle(clamped);
    }
    _currentGripperAngle = (deg < 0) ? 0 : (deg > 180) ? 180 : deg;
}

void Robot::zeroEncoders()
{
    _mc.resetEncoderAccumulators();
}

void Robot::setPose(int32_t x_mm, int32_t y_mm, int32_t h_cdeg)
{
    _odo.setPose(_state.inputs, x_mm, y_mm, h_cdeg);
}

void Robot::zeroOdometry()
{
    _odo.zero(_state.inputs);
}

// ---------------------------------------------------------------------------
// Query methods
// ---------------------------------------------------------------------------

Robot::EncoderReading Robot::getEncoders() const
{
    EncoderReading r{};
    _mc.getEncoderPositions(r.leftMm, r.rightMm);
    return r;
}

Robot::Pose Robot::getPose() const
{
    Pose p{};
    Odometry::getPose(_state.inputs, p.x_mm, p.y_mm, p.h_cdeg);
    return p;
}

// ---------------------------------------------------------------------------
// controlCollect — collect encoder readings and run the motor PID (014-003).
//
// 1. Calls Motor::collectEncoder() for each wheel and converts the raw
//    tenths-of-degrees reading to mm using the calibration scalars.
// 2. Writes the results to _state.inputs.encLMm / encRMm.
// 3. Computes dt_s from _lastControlMs; calls
//    _mc.controlTick(_state.inputs, _state.commands, dt_s).
//
// NOTE: Motor::collectEncoder() reads back the response to a prior
// Motor::requestEncoder() call.  In this stub path the requestEncoder()
// call that satisfies the split-phase contract is the synchronous read
// inside Motor::collectEncoder() itself (legacy path — no delay, same as
// before).  Correct split-phase timing is wired by the LoopScheduler in
// ticket 006; functional encoder reads are verified at bench in ticket 009.
// ---------------------------------------------------------------------------

void Robot::controlCollect(uint32_t now_ms)
{
    // Collect encoder readings using the split-phase API.
    //
    // In the stub path (before the LoopScheduler in ticket 006), we call
    // requestEncoder() immediately followed by collectEncoder() on each wheel
    // via readEncoderMmF() (which calls collectEncoder() internally).  This is
    // equivalent to the old readEncoderRaw() synchronous I2C transaction
    // (write then immediate read, no inter-transaction delay).
    //
    // The LoopScheduler in ticket 006 will insert the required ≥ one-loop-period
    // delay between the request and collect phases by alternating wheels across
    // ticks.  Until then, functional encoder reads are not guaranteed — build
    // correctness is the gate here; bench correctness is ticket 009.
    _motorL.requestEncoder();
    _state.inputs.encLMm = _motorL.readEncoderMmF(_config);
    _motorR.requestEncoder();
    _state.inputs.encRMm = _motorR.readEncoderMmF(_config);

    // Run PID + PWM via the new ZOH-aware signature.
    // In the sync (non-split-phase) path both wheels are updated on the same
    // tick, so there is no alternation. We pass refreshedWheel=0 (none) so
    // velocity is NOT recomputed from these synchronous reads (the reads happen
    // inside readEncoderMmF above, without proper inter-request delay, making
    // the delta unreliable). Velocity holds at zero until the LoopScheduler
    // split-phase path takes over.
    _lastControlMs = now_ms;
    _mc.controlTick(_state.inputs, _state.commands, now_ms, 0);
}

// ---------------------------------------------------------------------------
// odometryPredict — dead-reckoning update task entry point (014-004).
//
// Reads _state.inputs.encLMm / encRMm (written by controlCollect() this tick)
// and applies midpoint (exact-arc) integration into _state.inputs.poseX/Y/Hrad.
//
// This is the cooperative-loop task slot for odometry predict.  The LoopScheduler
// (ticket 006) will call this at the correct phase; until then controlTick() calls
// it directly after controlCollect().
// ---------------------------------------------------------------------------

void Robot::odometryPredict()
{
    _odo.predict(_state.inputs, _config.trackwidthMm);
}

// ---------------------------------------------------------------------------
// driveAdvance — drive state machine task entry point (014-005).
//
// Delegates to DriveController::driveAdvance(), passing the authoritative
// state structs.  EVT completions (done T/D/G, safety_stop) are emitted
// inline via _state.target.replyFn / replyCtx / corrId.
// ---------------------------------------------------------------------------

void Robot::driveAdvance(uint32_t now_ms)
{
    _dc.driveAdvance(_state.inputs, _state.commands, _state.target, now_ms);
}

// ---------------------------------------------------------------------------
// otosCorrect — OTOS complementary correction task entry point (014-004/005).
//
// If OTOS is present, reads raw position from hardware, converts LSB → mm/rad,
// applies the mounting-offset transform, writes _state.inputs.otosX/Y/H,
// updates otos.lastUpdMs, and calls Odometry::correct() on the struct.
//
// This is the SOLE OTOS correction path (014-005).  DriveController no longer
// has an OTOS block — it was removed in ticket 005.
// Called from controlTick() at the slow cadence (every ~100 ms via the
// kOtosSlowMs gate inside this method).  Ticket 006 will move it to a
// dedicated LoopScheduler task slot.
// ---------------------------------------------------------------------------

void Robot::otosCorrect(uint32_t now_ms)
{
    if (!_otos.is_initialized()) return;

    // Slow cadence gate: run correction at ~10 Hz (every kOtosSlowMs ms).
    // When called from a dedicated LoopScheduler slot (ticket 006), the
    // scheduler enforces the cadence and this gate can be removed.
    if ((now_ms - _lastOtosMs) < kOtosSlowMs) return;
    _lastOtosMs = now_ms;

    int16_t rx = 0, ry = 0, rh = 0;
    _otos.getPositionRaw(rx, ry, rh);

    constexpr float kPosMmPerLsb  = 0.305f;
    constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);

    float xF = static_cast<float>(rx) * kPosMmPerLsb;
    float yF = static_cast<float>(ry) * kPosMmPerLsb;
    float hF = static_cast<float>(rh) * kHdgRadPerLsb;

    if (_config.odomUpsideDown) {
        xF = -xF;
        yF = -yF;
        hF = -hF;
    }

    float angRad = -_config.odomYawDeg * (3.14159265f / 180.0f);
    float c = cosf(angRad);
    float s = sinf(angRad);

    _state.inputs.otosX = c * xF - s * yF - _config.odomOffX;
    _state.inputs.otosY = s * xF + c * yF - _config.odomOffY;
    _state.inputs.otosH = hF + _config.odomYawDeg * (3.14159265f / 180.0f);
    _state.inputs.otos.lastUpdMs = now_ms;
    _state.inputs.otos.valid     = true;

    _odo.correct(_state.inputs,
                 _state.inputs.otosX,
                 _state.inputs.otosY,
                 _state.inputs.otosH,
                 _config.alphaPos, _config.alphaYaw, _config.otosGate);
}

// ---------------------------------------------------------------------------
// controlFireRequest — fire the encoder request for the specified wheel (014-006).
//
// pendingWheel: 1 = left (M2), 2 = right (M1).
// Called by LoopScheduler as the LAST I2C operation before the idle sleep,
// keeping the motor's pending-read window free of other I2C traffic.
// ---------------------------------------------------------------------------

void Robot::controlFireRequest(int pendingWheel)
{
    if (pendingWheel == 1) {
        _motorL.requestEncoder();
    } else if (pendingWheel == 2) {
        _motorR.requestEncoder();
    }
}

// ---------------------------------------------------------------------------
// controlCollectSplitPhase — split-phase COLLECT for the cooperative loop (014-006).
//
// Reads back the encoder from the wheel indicated by pendingWheel (1=left,
// 2=right) using Motor::collectEncoder() + conversion, writes the result into
// _state.inputs.enc{L,R}Mm, then calls _mc.controlTick() for PID + PWM.
//
// If pendingWheel == 0 (first iteration, no prior request has been fired),
// the collect step is skipped — only the PID/PWM path runs so the motor
// controllers are warm before the first valid encoder reading arrives.
//
// The idle sleep in LoopScheduler::run() supplies the ≥ one-loop-period delay
// required between the requestEncoder() write and this collectEncoder() read.
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
    bool driving = (_state.commands.tgtLMms != 0.0f ||
                    _state.commands.tgtRMms != 0.0f);
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
            fmaxf(fabsf((float)_state.commands.tgtLMms),
                  fabsf((float)_state.commands.tgtRMms)) * 0.2f);
        static constexpr int   kRetries    = 2;

        // Right (M1) first — proven ordering from WedgeTest.
        {
            float newR = _motorR.readEncoderMmFSettle(_config);
            float dR   = newR - _state.inputs.encRMm;
            if (dR > kMaxDeltaMm || dR < -kMaxDeltaMm) {
                newR = _state.inputs.encRMm;             // default: hold old
                for (int k = 0; k < kRetries; ++k) {
                    float r2  = _motorR.readEncoderMmFSettle(_config);
                    float dr2 = r2 - _state.inputs.encRMm;
                    if (dr2 <= kMaxDeltaMm && dr2 >= -kMaxDeltaMm) { newR = r2; break; }
                }
            }
            _state.inputs.encRMm = newR;
        }

        // Left (M2) second.
        {
            float newL = _motorL.readEncoderMmFSettle(_config);
            float dL   = newL - _state.inputs.encLMm;
            if (dL > kMaxDeltaMm || dL < -kMaxDeltaMm) {
                newL = _state.inputs.encLMm;             // default: hold old
                for (int k = 0; k < kRetries; ++k) {
                    float r2  = _motorL.readEncoderMmFSettle(_config);
                    float dr2 = r2 - _state.inputs.encLMm;
                    if (dr2 <= kMaxDeltaMm && dr2 >= -kMaxDeltaMm) { newL = r2; break; }
                }
            }
            _state.inputs.encLMm = newL;
        }
    }
    _prevDriving = driving;
    _lastControlMs = now_ms;
    // refreshedWheel=3: both wheels updated; 0: idle, no velocity update.
    _mc.controlTick(_state.inputs, _state.commands, now_ms, driving ? 3 : 0);
}

// ---------------------------------------------------------------------------
// lineRead — read 4-channel line sensor into HardwareState (014-007).
//
// Writes _state.inputs.line[0..3]; updates lineVS.lastUpdMs and sets
// lineVS.valid on success.  No-op if line sensor is absent.
// ---------------------------------------------------------------------------

void Robot::lineRead()
{
    if (!_line.is_initialized()) return;
    if (_line.readValues(_state.inputs.line)) {
        _state.inputs.lineVS.lastUpdMs = systemTime();
        _state.inputs.lineVS.valid     = true;
    }
}

// ---------------------------------------------------------------------------
// colorRead — non-blocking RGBC poll into HardwareState (014-007).
//
// Writes _state.inputs.colorR/G/B/C; updates colorVS.lastUpdMs and sets
// colorVS.valid on success.  No-op if color sensor is absent.
// ---------------------------------------------------------------------------

void Robot::colorRead()
{
    if (!_color.is_initialized()) return;
    if (_color.pollRGBC(_state.inputs.colorR,
                        _state.inputs.colorG,
                        _state.inputs.colorB,
                        _state.inputs.colorC)) {
        _state.inputs.colorVS.lastUpdMs = systemTime();
        _state.inputs.colorVS.valid     = true;
    }
}

// ---------------------------------------------------------------------------
// portsRead — read digital and analogue GPIO ports into HardwareState (014-007).
// ---------------------------------------------------------------------------

void Robot::portsRead()
{
    for (uint8_t i = 0; i < 4; ++i) {
        _state.inputs.digitalIn[i] = (_portio.readDigital(i) != 0);
        _state.inputs.analogIn[i]  = (int16_t)_portio.readAnalog(i);
    }
    _state.inputs.portsVS.lastUpdMs = systemTime();
    _state.inputs.portsVS.valid     = true;
}

// ---------------------------------------------------------------------------
// telemetryEmit — assemble and emit the unified TLM frame (014-007).
//
// Replaces telemetryTick (now removed).  Reads ALL sensor data from
// _state.inputs (snapshots written by the sensor task entry points —
// lineRead, colorRead, portsRead, controlCollect).  No direct I2C calls.
//
// Emits when tlmPeriodMs has elapsed or a SNAP is pending.
// ---------------------------------------------------------------------------

// buildTlmFrame — assemble the TLM line into buf; returns its length. Shared by
// the periodic STREAM (telemetryEmit) and the synchronous telemetry REQUEST (the
// SNAP command, which replies with this frame). Honors the tlmFields mask.
int Robot::buildTlmFrame(char* buf, int len)
{
    uint32_t t_sample = systemTime();
    int32_t encL = static_cast<int32_t>(_state.inputs.encLMm);
    int32_t encR = static_cast<int32_t>(_state.inputs.encRMm);

    int32_t pose_x = 0, pose_y = 0, pose_h = 0;
    if (_config.tlmFields & TLM_FIELD_POSE) {
        Odometry::getPose(_state.inputs, pose_x, pose_y, pose_h);
    }
    bool haveLine = _line.is_initialized() && _state.inputs.lineVS.valid &&
                    (_config.tlmFields & TLM_FIELD_LINE);
    bool haveColor = _color.is_initialized() && _state.inputs.colorVS.valid &&
                     (_config.tlmFields & TLM_FIELD_COLOR);
    bool haveVel = (_config.tlmFields & TLM_FIELD_VEL) != 0;
    float velL = haveVel ? _state.inputs.velLMms : 0.0f;
    float velR = haveVel ? _state.inputs.velRMms : 0.0f;

    char modeChar = 'I';
    switch (_dc.mode()) {
        case DriveMode::STREAMING: modeChar = 'S'; break;
        case DriveMode::TIMED:     modeChar = 'T'; break;
        case DriveMode::DISTANCE:  modeChar = 'D'; break;
        case DriveMode::GO_TO:     modeChar = 'G'; break;
        default:                   modeChar = 'I'; break;
    }

    int pos = 0, rem = len;
    int n = snprintf(buf + pos, (size_t)rem, "TLM t=%lu mode=%c",
                     (unsigned long)t_sample, modeChar);
    if (n > 0 && n < rem) { pos += n; rem -= n; }
    if (_config.tlmFields & TLM_FIELD_ENC) {
        n = snprintf(buf + pos, (size_t)rem, " enc=%d,%d", (int)encL, (int)encR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (_config.tlmFields & TLM_FIELD_POSE) {
        n = snprintf(buf + pos, (size_t)rem, " pose=%d,%d,%d",
                     (int)pose_x, (int)pose_y, (int)pose_h);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveVel) {
        n = snprintf(buf + pos, (size_t)rem, " vel=%d,%d", (int)velL, (int)velR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveLine) {
        n = snprintf(buf + pos, (size_t)rem, " line=%u,%u,%u,%u",
                     (unsigned)_state.inputs.line[0], (unsigned)_state.inputs.line[1],
                     (unsigned)_state.inputs.line[2], (unsigned)_state.inputs.line[3]);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveColor) {
        n = snprintf(buf + pos, (size_t)rem, " color=%u,%u,%u,%u",
                     (unsigned)_state.inputs.colorR, (unsigned)_state.inputs.colorG,
                     (unsigned)_state.inputs.colorB, (unsigned)_state.inputs.colorC);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    buf[pos] = '\0';
    return pos;
}

void Robot::telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    // STREAM telemetry only while the motors are running (+ a short grace to flush
    // the last frame). When stopped, the stream goes silent so the radio link is
    // clear for commands. To read telemetry while stopped, the host REQUESTS it
    // (the SNAP command returns a frame synchronously — see CommandProcessor).
    static constexpr uint32_t kGraceMs = 400;
    if (_dc.mode() != DriveMode::IDLE) _lastActiveMs = now_ms;
    bool stopped = (now_ms - _lastActiveMs) > kGraceMs;

    bool periodic = (_config.tlmPeriodMs > 0) && !stopped &&
                    ((now_ms - _lastTlmMs) >= (uint32_t)_config.tlmPeriodMs);
    if (!periodic) return;

    if (_config.tlmPeriodMs < 20) _config.tlmPeriodMs = 20;  // clamp to 50 Hz max

    char tlmBuf[128];
    buildTlmFrame(tlmBuf, sizeof(tlmBuf));
    fn(tlmBuf, ctx);
    _lastTlmMs = now_ms;
}
