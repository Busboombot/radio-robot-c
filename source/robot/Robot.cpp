#include "Robot.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "DriveController.h"
#include <cstdio>
#include <cmath>

// ---------------------------------------------------------------------------
// DIAGNOSTIC TOGGLE (Sprint 014) — OTOS <-> color I2C bus-conflict test.
//
// When set to 1, the firmware NEVER drives any I2C traffic to the OTOS:
// boot detection (OtosSensor::begin, which reads REG_PRODUCT_ID at 0x17) is
// skipped and _otosPresent is forced false, so init/setLinearScalar/
// setAngularScalar never run; and Robot::otosCorrect() early-returns before
// any I2C transaction (getPositionRaw at 0x17).  The OTOS can stay physically
// on the bus while remaining completely untouched by firmware, so we can
// isolate whether our ACTIVE OTOS reads (vs a passive electrical conflict)
// are what wedge the bus / break encoder reads when the color sensor (0x43)
// and OTOS (0x17) are connected together.
//
// Leave color, line, encoders, motors, ports, etc. fully active.  Set back
// to 0 to restore normal OTOS behaviour.
// ---------------------------------------------------------------------------
#define DISABLE_OTOS_SENSOR 1

Robot::Robot(MicroBitI2C&    i2c,
             NRF52Serial&    serial,
             MicroBitRadio&  radio,
             MicroBitIO&     io,
             MessageBus&     messageBus,
             MicroBit&       uBit)
    : _uBit(uBit),
      _currentGripperAngle(0),
      _config(defaultRobotConfig()),
      _lastTlmMs(0),
      _motorL(i2c, 2, _config.fwdSignL),  // M2, left wheel
      _motorR(i2c, 1, _config.fwdSignR),   // M1, right wheel
      _serial(serial),
      _radio(radio, messageBus),
      _otos(i2c),
      _otosPresent(false),
      _line(i2c),
      _linePresent(false),
      _color(i2c),
      _colorPresent(false),
      _servo(io.P1),
      _gripperPresent(false),
      _portio(io),
      _mc(_motorL, _motorR, _config),
      _odo(),
      _dc(_mc, _odo, _config),
      _state(defaultInputs(_config)),
      _lastControlMs(0),
      _lastOtosMs(0)
{
    // uBit.init() was called by main.cpp before constructing Robot.
    // All CODAL peripherals are ready; begin subsystem initialisation now.

    _serial.begin();
    _radio.begin();

    // Probe optional sensors; mark absent if hardware not connected.
#if DISABLE_OTOS_SENSOR
    // Diagnostic: skip OTOS detection entirely — no I2C probe/read of 0x17.
    _otosPresent = false;
#else
    _otosPresent = _otos.begin();
#endif
    if (_otosPresent) {
        _otos.init();
        // OTOS correction is handled by Robot::otosCorrect() exclusively (014-005).
        // DriveController no longer holds the OtosSensor pointer.

        // Apply calibration scalars from config at boot.
        // Formula: scalar = clamp(round((scale - 1.0) / 0.001), -127, 127).
        // E.g. otosLinearScale=1.05 → +50; otosAngularScale=0.987 → -13.
        auto scaleToInt8 = [](float scale) -> int8_t {
            float raw = roundf((scale - 1.0f) / 0.001f);
            if (raw > 127.0f) raw = 127.0f;
            if (raw < -127.0f) raw = -127.0f;
            return static_cast<int8_t>(raw);
        };
        _otos.setLinearScalar(scaleToInt8(_config.otosLinearScale));
        _otos.setAngularScalar(scaleToInt8(_config.otosAngularScale));
    }

    _linePresent  = _line.readValues(nullptr);  // probe: returns false on I2C error
    _colorPresent = _color.begin();
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
// Drive action methods — delegate to DriveController
// ---------------------------------------------------------------------------

void Robot::stop()
{
    uint32_t now_ms = _uBit.systemTime();
    // stop() with no reply fn: use a no-op sink
    _dc.stop(now_ms, [](const char*, void*){}, nullptr);
}

void Robot::streamDrive(int32_t leftMms, int32_t rightMms, ReplyFn fn, void* ctx)
{
    _dc.beginStream((float)leftMms, (float)rightMms, _uBit.systemTime(),
                    _state.target, fn, ctx);
}

void Robot::velocityDrive(float v_mms, float omega_rads, ReplyFn fn, void* ctx,
                           const char* corr_id)
{
    _dc.beginVelocity(v_mms, omega_rads, _uBit.systemTime(),
                      _state.target, fn, ctx, corr_id);
}

void Robot::timedDrive(int32_t leftMms, int32_t rightMms, uint32_t durationMs,
                       ReplyFn fn, void* ctx, const char* corr_id)
{
    _dc.beginTimed((float)leftMms, (float)rightMms, durationMs, _uBit.systemTime(),
                   _state.target, fn, ctx, corr_id);
}

void Robot::distanceDrive(int32_t leftMms, int32_t rightMms, int32_t targetMm,
                          ReplyFn fn, void* ctx, const char* corr_id)
{
    _dc.beginDistance((float)leftMms, (float)rightMms, targetMm, _uBit.systemTime(),
                      _state.target, fn, ctx, corr_id);
}

void Robot::goTo(float tx, float ty, float speedMms, ReplyFn fn, void* ctx,
                 const char* corr_id)
{
    _dc.beginGoTo(tx, ty, speedMms, _uBit.systemTime(),
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
#if DISABLE_OTOS_SENSOR
    // Diagnostic: never issue OTOS I2C (getPositionRaw at 0x17) from the
    // runtime correction task.
    (void)now_ms;
    return;
#endif
    if (!_otosPresent) return;

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

void Robot::controlCollectSplitPhase(uint32_t now_ms, int pendingWheel)
{
    // Atomic per-tick encoder read (fix[014]: restores closed-loop velocity).
    //
    // Root cause: the split-phase design fired requestEncoder() at the END of
    // tick N, slept ~10 ms (the control period), then called collectEncoder()
    // at the TOP of tick N+1.  The Nezha V2 chip's 0x46 response window is only
    // ~4 ms (vendor-documented); the ~10 ms cross-iteration gap causes
    // collectEncoder() to always return a stale/zero value, leaving enc=0,0
    // throughout a drive and saturating the velocity PID to constant PWM.
    //
    // Fix: call readEncoderMmFAtomic() which uses the full vendor timing:
    //   4ms pre-write bus-idle → 0x46 write → 4ms post-write settle → read 4B
    // (matches sprint 013 readEncoderRaw() which had both delays and worked).
    //
    // Cost: ~8 ms per tick.  controlPeriodMs must be ≥ 10 ms.  Alternating L/R
    // refreshes each wheel at half the control rate (~5 Hz at 10 ms).
    //
    // On pendingWheel == 0 (first iteration): no read; encoders remain
    // 0-initialised and refreshedWheel=0 suppresses velocity update.

    if (pendingWheel == 1) {
        // Refresh left wheel using the full atomic read (pre-delay + write +
        // post-delay + read), matching the sprint 013 readEncoderRaw() timing.
        _state.inputs.encLMm = _motorL.readEncoderMmFAtomic(_config);
    } else if (pendingWheel == 2) {
        // Refresh right wheel.
        _state.inputs.encRMm = _motorR.readEncoderMmFAtomic(_config);
    }
    // pendingWheel == 0: first iteration — skip; encoder fields remain
    // 0-initialised from defaultInputs(). PID runs with ZOH velocity = 0.

    _lastControlMs = now_ms;

    // Pass the pendingWheel that was just READ as refreshedWheel so that
    // MotorController updates that wheel's per-wheel velocity using the correct
    // elapsed time since the last collect for that wheel.
    _mc.controlTick(_state.inputs, _state.commands, now_ms, pendingWheel);
}

// ---------------------------------------------------------------------------
// lineRead — read 4-channel line sensor into HardwareState (014-007).
//
// Writes _state.inputs.line[0..3]; updates lineVS.lastUpdMs and sets
// lineVS.valid on success.  No-op if line sensor is absent.
// ---------------------------------------------------------------------------

void Robot::lineRead()
{
    if (!_linePresent) return;
    if (_line.readValues(_state.inputs.line)) {
        _state.inputs.lineVS.lastUpdMs = _uBit.systemTime();
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
    if (!_colorPresent) return;
    if (_color.pollRGBC(_state.inputs.colorR,
                        _state.inputs.colorG,
                        _state.inputs.colorB,
                        _state.inputs.colorC)) {
        _state.inputs.colorVS.lastUpdMs = _uBit.systemTime();
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
    _state.inputs.portsVS.lastUpdMs = _uBit.systemTime();
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

void Robot::telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    bool snapPending = _config.tlmSnapPending;
    bool periodic    = (_config.tlmPeriodMs > 0) &&
                       ((now_ms - _lastTlmMs) >= (uint32_t)_config.tlmPeriodMs);

    if (!snapPending && !periodic) return;

    // Clamp period to 20 ms minimum to avoid flooding the buffer.
    if (periodic && _config.tlmPeriodMs < 20) {
        _config.tlmPeriodMs = 20;
    }

    // ----- 1. Capture timestamp -----------------------------------------------
    uint32_t t_sample = _uBit.systemTime();

    // ----- 2. Encoder positions from HardwareState snapshot -------------------
    int32_t encL = static_cast<int32_t>(_state.inputs.encLMm);
    int32_t encR = static_cast<int32_t>(_state.inputs.encRMm);

    // ----- 3. Pose from HardwareState -----------------------------------------
    int32_t pose_x = 0, pose_y = 0, pose_h = 0;
    if (_config.tlmFields & TLM_FIELD_POSE) {
        Odometry::getPose(_state.inputs, pose_x, pose_y, pose_h);
    }

    // ----- 4. Line from HardwareState snapshot --------------------------------
    bool haveLine = _linePresent && _state.inputs.lineVS.valid &&
                    (_config.tlmFields & TLM_FIELD_LINE);

    // ----- 5. Color from HardwareState snapshot -------------------------------
    bool haveColor = _colorPresent && _state.inputs.colorVS.valid &&
                     (_config.tlmFields & TLM_FIELD_COLOR);

    // ----- 6. Velocity from HardwareState -------------------------------------
    float velL = 0.0f, velR = 0.0f;
    bool haveVel = false;
    if (_config.tlmFields & TLM_FIELD_VEL) {
        velL    = _state.inputs.velLMms;
        velR    = _state.inputs.velRMms;
        haveVel = true;
    }

    // ----- 7. Drive mode character --------------------------------------------
    char modeChar = 'I';
    switch (_dc.mode()) {
        case DriveMode::STREAMING: modeChar = 'S'; break;
        case DriveMode::TIMED:     modeChar = 'T'; break;
        case DriveMode::DISTANCE:  modeChar = 'D'; break;
        case DriveMode::GO_TO:     modeChar = 'G'; break;
        default:                   modeChar = 'I'; break;
    }

    // ----- 8. Assemble TLM line -----------------------------------------------
    char tlmBuf[128];
    int  pos = 0;
    int  rem = (int)sizeof(tlmBuf);

    int n = snprintf(tlmBuf + pos, (size_t)rem,
                     "TLM t=%lu mode=%c",
                     (unsigned long)t_sample, modeChar);
    if (n > 0 && n < rem) { pos += n; rem -= n; }

    if (_config.tlmFields & TLM_FIELD_ENC) {
        n = snprintf(tlmBuf + pos, (size_t)rem, " enc=%d,%d", (int)encL, (int)encR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }

    if (_config.tlmFields & TLM_FIELD_POSE) {
        n = snprintf(tlmBuf + pos, (size_t)rem,
                     " pose=%d,%d,%d", (int)pose_x, (int)pose_y, (int)pose_h);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }

    if (haveVel) {
        n = snprintf(tlmBuf + pos, (size_t)rem,
                     " vel=%d,%d", (int)velL, (int)velR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }

    if (haveLine) {
        n = snprintf(tlmBuf + pos, (size_t)rem,
                     " line=%u,%u,%u,%u",
                     (unsigned)_state.inputs.line[0],
                     (unsigned)_state.inputs.line[1],
                     (unsigned)_state.inputs.line[2],
                     (unsigned)_state.inputs.line[3]);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }

    if (haveColor) {
        n = snprintf(tlmBuf + pos, (size_t)rem,
                     " color=%u,%u,%u,%u",
                     (unsigned)_state.inputs.colorR,
                     (unsigned)_state.inputs.colorG,
                     (unsigned)_state.inputs.colorB,
                     (unsigned)_state.inputs.colorC);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }

    tlmBuf[pos] = '\0';

    // ----- 9. Emit and update state ------------------------------------------
    fn(tlmBuf, ctx);
    _lastTlmMs = now_ms;
    _config.tlmSnapPending = false;
}
