#include "Robot.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "DriveController.h"
#include <cstdio>
#include <cmath>

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
      _dc(_mc, _odo, _config),  // OTOS pointer set below after hardware probe
      _state(defaultInputs(_config)),
      _lastControlMs(0)
{
    // uBit.init() was called by main.cpp before constructing Robot.
    // All CODAL peripherals are ready; begin subsystem initialisation now.

    _serial.begin();
    _radio.begin();

    // Probe optional sensors; mark absent if hardware not connected.
    _otosPresent = _otos.begin();
    if (_otosPresent) {
        _otos.init();
        _dc.setOtos(&_otos);  // wire OTOS into DriveController for fusion

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

    // Bind authoritative HardwareState into DriveController so it can call
    // Odometry::correct() with the struct-based API (014-004).
    // Ticket 005 will move correct() into Robot::otosCorrect() and remove
    // this coupling.
    _dc.setHardwareState(&_state.inputs);

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
    _dc.beginStream((float)leftMms, (float)rightMms, _uBit.systemTime(), fn, ctx);
}

void Robot::velocityDrive(float v_mms, float omega_rads, ReplyFn fn, void* ctx,
                           const char* corr_id)
{
    _dc.beginVelocity(v_mms, omega_rads, _uBit.systemTime(), fn, ctx, corr_id);
}

void Robot::timedDrive(int32_t leftMms, int32_t rightMms, uint32_t durationMs,
                       ReplyFn fn, void* ctx, const char* corr_id)
{
    _dc.beginTimed((float)leftMms, (float)rightMms, durationMs, _uBit.systemTime(), fn, ctx, corr_id);
}

void Robot::distanceDrive(int32_t leftMms, int32_t rightMms, int32_t targetMm,
                          ReplyFn fn, void* ctx, const char* corr_id)
{
    _dc.beginDistance((float)leftMms, (float)rightMms, targetMm, _uBit.systemTime(), fn, ctx, corr_id);
}

void Robot::goTo(float tx, float ty, float speedMms, ReplyFn fn, void* ctx,
                 const char* corr_id)
{
    _dc.beginGoTo(tx, ty, speedMms, _uBit.systemTime(), fn, ctx, corr_id);
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
// controlTick — control fiber entry point (013-010).
//
// Runs at a fixed period (RobotConfig::controlPeriodMs, default 10 ms).
// Only the PID/motor/odometry path runs here.  No serial/radio I/O.
// ---------------------------------------------------------------------------

void Robot::controlTick(uint32_t now_ms)
{
    // Collect encoder readings + run motor PID.
    controlCollect(now_ms);

    // Dead-reckoning update: reads encLMm/R from _state.inputs, writes poseX/Y/Hrad.
    // (014-004: odometry predict moved from DriveController to Robot.)
    odometryPredict();

    // Advance drive-mode state machines (S/T/D/G + OTOS correction).
    // The cooperative-loop split (ticket 006) will later call these from the
    // LoopScheduler at the correct phase.
    _dc.controlTick(now_ms);
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

    // Compute dt_s; guard against zero on the first call.
    float dt_s = 0.0f;
    if (_lastControlMs != 0) {
        dt_s = static_cast<float>(now_ms - _lastControlMs) / 1000.0f;
    }
    _lastControlMs = now_ms;

    _mc.controlTick(_state.inputs, _state.commands, dt_s);
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
// otosCorrect — OTOS complementary correction task entry point (014-004).
//
// If OTOS is present, reads raw position from hardware, converts LSB → mm/rad,
// applies the mounting-offset transform, writes _state.inputs.otosX/Y/H,
// updates otos.lastUpdMs, and calls Odometry::correct() on the struct.
//
// This method provides the clean implementation for the otos-correct task slot
// (ticket 005 wires this from the cooperative loop and removes the OTOS block
// from DriveController::controlTick()).  Until ticket 005, the OTOS correction
// in DriveController::controlTick() remains active.
//
// IMPORTANT: Do NOT call this from controlTick() until ticket 005 removes the
// DriveController OTOS block — calling both would double-apply the correction.
// ---------------------------------------------------------------------------

void Robot::otosCorrect(uint32_t now_ms)
{
    if (!_otosPresent) return;

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
// telemetryTick — comms+telemetry fiber entry point (013-010).
//
// 1. Drain any pending EVT completions from DriveController (safety_stop,
//    done T/D/G) and emit them via fn/ctx.
// 2. Assemble and emit one unified TLM frame when the configured period has
//    elapsed, or immediately if a SNAP was requested.
//
// Reads only cached encoder/velocity snapshots from MotorController (which
// the control fiber updates).  Line/color I2C is safe here because Motor
// I2C is now atomic (busy-wait prevents interleave).
// ---------------------------------------------------------------------------

void Robot::telemetryTick(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    // 1. Drain EVT completions enqueued by the control fiber.
    _dc.drainEvents(fn, ctx);

    // 2. TLM assembly ---------------------------------------------------------
    // Emit one unified TLM frame when the configured period has elapsed, or
    // immediately if a SNAP was requested.  t= is stamped at sensor-read time,
    // not at snprintf time, to avoid send-latency bias.

    bool snapPending = _config.tlmSnapPending;
    bool periodic    = (_config.tlmPeriodMs > 0) &&
                       ((now_ms - _lastTlmMs) >= (uint32_t)_config.tlmPeriodMs);

    if (!snapPending && !periodic) return;

    // Clamp period to 20 ms minimum to avoid flooding the buffer.
    if (periodic && _config.tlmPeriodMs < 20) {
        _config.tlmPeriodMs = 20;
    }

    // ----- 1. Capture timestamp at sensor-read time -------------------------
    uint32_t t_sample = _uBit.systemTime();

    // ----- 2. Read encoder positions ----------------------------------------
    int32_t encL = 0, encR = 0;
    _mc.getEncoderPositions(encL, encR);

    // ----- 3. Read pose — always fused odometry (mm, mm, centidegrees) ------
    // Authoritative pose now lives in _state.inputs.poseX/Y/Hrad (014-004).
    int32_t pose_x = 0, pose_y = 0, pose_h = 0;
    if (_config.tlmFields & TLM_FIELD_POSE) {
        Odometry::getPose(_state.inputs, pose_x, pose_y, pose_h);
    }

    // ----- 4. Read line sensor (if present and field requested) --------------
    uint16_t lineVals[4] = {0, 0, 0, 0};
    bool haveLine = false;
    if (_linePresent && (_config.tlmFields & TLM_FIELD_LINE)) {
        haveLine = _line.readValues(lineVals);
    }

    // ----- 5. Read color sensor (if present and field requested) -------------
    uint16_t colorR = 0, colorG = 0, colorB = 0, colorC = 0;
    bool haveColor = false;
    if (_colorPresent && (_config.tlmFields & TLM_FIELD_COLOR)) {
        haveColor = _color.pollRGBC(colorR, colorG, colorB, colorC);
    }

    // ----- 6. Read velocity from HardwareState (encoder-delta, always available) ----
    float velL = 0.0f, velR = 0.0f;
    bool haveVel = false;
    if (_config.tlmFields & TLM_FIELD_VEL) {
        velL = _state.inputs.velLMms;
        velR = _state.inputs.velRMms;
        haveVel = true;
    }

    // ----- 7. Determine drive mode character ---------------------------------
    char modeChar = 'I';
    switch (_dc.mode()) {
        case DriveMode::STREAMING: modeChar = 'S'; break;
        case DriveMode::TIMED:     modeChar = 'T'; break;
        case DriveMode::DISTANCE:  modeChar = 'D'; break;
        case DriveMode::GO_TO:     modeChar = 'G'; break;
        default:                   modeChar = 'I'; break;
    }

    // ----- 8. Assemble TLM line (~90 bytes) ----------------------------------
    char tlmBuf[128];
    int  pos = 0;
    int  rem = (int)sizeof(tlmBuf);

    // TLM header: tag + timestamp + mode
    int n = snprintf(tlmBuf + pos, (size_t)rem,
                     "TLM t=%lu mode=%c",
                     (unsigned long)t_sample, modeChar);
    if (n > 0 && n < rem) { pos += n; rem -= n; }

    // enc= field
    if (_config.tlmFields & TLM_FIELD_ENC) {
        n = snprintf(tlmBuf + pos, (size_t)rem, " enc=%d,%d", (int)encL, (int)encR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }

    // pose= field (always emitted when requested; odometry is always available)
    if (_config.tlmFields & TLM_FIELD_POSE) {
        n = snprintf(tlmBuf + pos, (size_t)rem,
                     " pose=%d,%d,%d", (int)pose_x, (int)pose_y, (int)pose_h);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }

    // vel= field
    if (haveVel) {
        n = snprintf(tlmBuf + pos, (size_t)rem,
                     " vel=%d,%d", (int)velL, (int)velR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }

    // line= field
    if (haveLine) {
        n = snprintf(tlmBuf + pos, (size_t)rem,
                     " line=%u,%u,%u,%u",
                     (unsigned)lineVals[0], (unsigned)lineVals[1],
                     (unsigned)lineVals[2], (unsigned)lineVals[3]);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }

    // color= field
    if (haveColor) {
        n = snprintf(tlmBuf + pos, (size_t)rem,
                     " color=%u,%u,%u,%u",
                     (unsigned)colorR, (unsigned)colorG,
                     (unsigned)colorB, (unsigned)colorC);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }

    tlmBuf[pos] = '\0';

    // ----- 9. Emit and update state ------------------------------------------
    fn(tlmBuf, ctx);
    _lastTlmMs = now_ms;
    _config.tlmSnapPending = false;
}
