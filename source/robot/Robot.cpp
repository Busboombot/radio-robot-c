#include "Robot.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "DriveController.h"
#include <cstdio>

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
      _dc(_mc, _odo, _config)
{
    // uBit.init() was called by main.cpp before constructing Robot.
    // All CODAL peripherals are ready; begin subsystem initialisation now.

    _serial.begin();
    _radio.begin();

    // Probe optional sensors; mark absent if hardware not connected.
    _otosPresent = _otos.begin();
    if (_otosPresent) _otos.init();

    _linePresent  = _line.readValues(nullptr);  // probe: returns false on I2C error
    _colorPresent = _color.begin();
    _gripperPresent = true;  // servo always available on P1

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
    _odo.setPose(x_mm, y_mm, h_cdeg);
}

void Robot::zeroOdometry()
{
    _odo.zero();
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
    _odo.getPose(p.x_mm, p.y_mm, p.h_cdeg);
    return p;
}

// ---------------------------------------------------------------------------
// tick — advance all subsystems; no while loop inside.
// fn/ctx: active reply sink (for streaming telemetry).
// Per-drive async completions use the captured per-drive sink.
// ---------------------------------------------------------------------------

void Robot::tick(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    _dc.tick(now_ms, fn, ctx);

    // ---- TLM assembly -------------------------------------------------------
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

    // ----- 3. Read pose (OTOS if present, else odometry) --------------------
    int32_t pose_x = 0, pose_y = 0, pose_h = 0;
    if (_config.tlmFields & TLM_FIELD_POSE) {
        if (_otosPresent) {
            int16_t rx = 0, ry = 0, rh = 0;
            _otos.getPositionRaw(rx, ry, rh);
            pose_x = (int32_t)rx;
            pose_y = (int32_t)ry;
            pose_h = (int32_t)rh;
        } else {
            _odo.getPose(pose_x, pose_y, pose_h);
        }
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

    // ----- 6. Read velocity (encoder-delta, always available) ----------------
    float velL = 0.0f, velR = 0.0f;
    bool haveVel = false;
    if (_config.tlmFields & TLM_FIELD_VEL) {
        _mc.getActualVelocity(velL, velR);
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
