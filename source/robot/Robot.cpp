#include "Robot.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include <cstdio>

Robot::Robot(MicroBitI2C&    i2c,
             NRF52Serial&    serial,
             MicroBitRadio&  radio,
             MicroBitIO&     io,
             MessageBus&     messageBus,
             MicroBit&       uBit)
    : _uBit(uBit),
      _currentGripperAngle(0),
      _motor(i2c),
      _serial(serial),
      _radio(radio, messageBus),
      _announcer(uBit, _serial, _radio),
      _config(defaultRobotConfig()),
      _otos(i2c),
      _otosPresent(false),
      _line(i2c),
      _linePresent(false),
      _color(i2c),
      _colorPresent(false),
      _gripper(io.P1),
      _gripperPresent(false),
      _portio(io),
      _mc(_motor, _config),
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

    // Register sensor streaming callback with DriveController so that CS/LS
    // readings are emitted alongside encoder reports during active drives.
    _dc.setSensorReporter(Robot::sensorReport, this);

    // Emit initial announcement so the host can detect the device.
    _announcer.announce();
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
                       ReplyFn fn, void* ctx)
{
    _dc.beginTimed((float)leftMms, (float)rightMms, durationMs, _uBit.systemTime(), fn, ctx);
}

void Robot::distanceDrive(int32_t leftMms, int32_t rightMms, int32_t targetMm,
                          ReplyFn fn, void* ctx)
{
    _dc.beginDistance((float)leftMms, (float)rightMms, targetMm, _uBit.systemTime(), fn, ctx);
}

void Robot::goTo(float tx, float ty, float speedMms, ReplyFn fn, void* ctx)
{
    _dc.beginGoTo(tx, ty, speedMms, _uBit.systemTime(), fn, ctx);
}

// ---------------------------------------------------------------------------
// Non-drive action methods
// ---------------------------------------------------------------------------

void Robot::setGripperAngle(int32_t deg)
{
    if (_gripperPresent) {
        uint8_t clamped = (deg < 0) ? 0 : (deg > 180) ? 180 : (uint8_t)deg;
        _gripper.setAngle(clamped);
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
// Sensor streaming callback — invoked by DriveController during drive ticks.
// Emits CS and LS readings (if sensors present) through the active reply sink.
// ---------------------------------------------------------------------------

void Robot::sensorReport(ReplyFn fn, void* ctx, void* sensorCtx)
{
    Robot* self = static_cast<Robot*>(sensorCtx);

    if (self->_colorPresent) {
        uint16_t cr = 0, cg = 0, cb = 0, cc = 0;
        self->_color.readRGBC(cr, cg, cb, cc);
        char buf[48];
        snprintf(buf, sizeof(buf), "CS%+d%+d%+d%+d",
                 (int)cr, (int)cg, (int)cb, (int)cc);
        fn(buf, ctx);
    }

    if (self->_linePresent) {
        uint16_t out[4] = {0, 0, 0, 0};
        self->_line.readValues(out);
        char buf[48];
        snprintf(buf, sizeof(buf), "LS%+d%+d%+d%+d",
                 (int)out[0], (int)out[1], (int)out[2], (int)out[3]);
        fn(buf, ctx);
    }
}

// ---------------------------------------------------------------------------
// tick — advance all subsystems; no while loop inside.
// fn/ctx: active reply sink (for streaming telemetry — encoder, CS, LS).
// Per-drive async completions use the captured per-drive sink.
// ---------------------------------------------------------------------------

void Robot::tick(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    _dc.tick(now_ms, fn, ctx);
}
