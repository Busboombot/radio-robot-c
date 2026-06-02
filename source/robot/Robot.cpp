#include "Robot.h"

static void serialReply(const char* msg, void* ctx) {
    static_cast<SerialPort*>(ctx)->send(msg);
}

static void radioReply(const char* msg, void* ctx) {
    static_cast<Radio*>(ctx)->send(msg);
}

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
      _cmd()
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

    // Wire hardware pointers into the command processor.
    _cmd.init(&_motor, &_mc, &_odo,
              _otosPresent ? &_otos : nullptr,
              _linePresent ? &_line : nullptr,
              _colorPresent ? &_color : nullptr,
              _gripperPresent ? &_gripper : nullptr,
              &_portio);
    _cmd.setConfig(&_config);
}

void Robot::run() {
    while (true) {
        // Direct serial commands — reply over serial.
        while (_serial.readLine(_buf, sizeof(_buf))) {
            if (!_announcer.handle(_buf, serialReply, &_serial)) {
                _cmd.process(_buf, serialReply, &_serial);
            }
        }
        // Commands via the RadioRelay (RAW250) — reply over the radio, which the
        // relay forwards back to the host serial port.
        while (_radio.poll(_buf, sizeof(_buf))) {
            if (!_announcer.handle(_buf, radioReply, &_radio)) {
                _cmd.process(_buf, radioReply, &_radio);
            }
        }
        _cmd.tick(_uBit.systemTime(), serialReply, &_serial);
    }
}
