#include "Robot.h"

Robot::Robot(MicroBitI2C&    i2c,
             NRF52Serial&    serial,
             MicroBitRadio&  radio,
             MicroBitIO&     io,
             MessageBus&     messageBus,
             MicroBit&       uBit)
    : _uBit(uBit),
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

void Robot::streamDrive(int32_t leftMms, int32_t rightMms)
{
    _dc.beginStream((float)leftMms, (float)rightMms, _uBit.systemTime());
}

void Robot::timedDrive(int32_t leftMms, int32_t rightMms, uint32_t durationMs)
{
    _dc.beginTimed((float)leftMms, (float)rightMms, durationMs, _uBit.systemTime());
}

void Robot::distanceDrive(int32_t leftMms, int32_t rightMms, int32_t targetMm)
{
    _dc.beginDistance((float)leftMms, (float)rightMms, targetMm, _uBit.systemTime());
}

void Robot::goTo(float tx, float ty, float speedMms)
{
    _dc.beginGoTo(tx, ty, speedMms, _uBit.systemTime());
}

// ---------------------------------------------------------------------------
// tick — advance all subsystems; no while loop inside.
// fn/ctx must be the active reply sink (set by the main loop to whichever
// channel delivered the most recent command), so async completions
// (T+DONE, D+DONE, G+DONE, SAFETY_STOP) return to the originating channel.
// ---------------------------------------------------------------------------

void Robot::tick(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    _dc.tick(now_ms, fn, ctx);
}
