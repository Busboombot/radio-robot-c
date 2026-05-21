#include "Robot.h"

static void serialReply(const char* msg, void* ctx) {
    static_cast<SerialPort*>(ctx)->send(msg);
}

static void radioReply(const char* msg, void* ctx) {
    static_cast<Radio*>(ctx)->send(msg);
}

Robot::Robot()
    : uBit(),
      _motor(uBit.i2c),
      _serial(uBit.serial),
      _radio(uBit.radio, uBit.messageBus),
      _announcer(uBit, _serial, _radio),
      _cal(defaultCalibParams()),
      _otos(uBit.i2c),
      _otosPresent(false),
      _line(uBit.i2c),
      _linePresent(false),
      _color(uBit.i2c),
      _colorPresent(false),
      _gripper(uBit.io.P1),
      _gripperPresent(false),
      _portio(uBit.io),
      _mc(_motor, _cal),
      _odo(),
      _cmd()
{
    // uBit.init() MUST be called before any subsystem initialization.
    // Member initialization above only stores references — actual I2C
    // communication begins below after the CODAL runtime is ready.
    uBit.init();

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
    _cmd.setCalib(&_cal);
}

void Robot::run() {
    bool isRelayed;
    while (true) {
        while (_serial.readLine(_buf, sizeof(_buf))) {
            if (!_announcer.handle(_buf)) {
                _cmd.process(_buf, serialReply, &_serial);
            }
        }
        while (_radio.poll(_buf, sizeof(_buf), isRelayed)) {
            if (!_announcer.handle(_buf)) {
                _cmd.process(_buf, radioReply, &_radio);
            }
        }
        _cmd.tick(uBit.systemTime(), serialReply, &_serial);
        uBit.sleep(20);
    }
}
