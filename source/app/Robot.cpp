#include "app/Robot.h"

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
      _portio(uBit.io)
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
}

void Robot::run() {
    bool isRelayed;
    while (true) {
        while (_serial.readLine(_buf, sizeof(_buf))) {
            if (!_announcer.handle(_buf)) {
                // sprint 2: _cmd.dispatch(_buf, ...)
            }
        }
        while (_radio.poll(_buf, sizeof(_buf), isRelayed)) {
            if (!_announcer.handle(_buf)) {
                // sprint 2: _cmd.dispatch(_buf, ...)
            }
        }
        // sprint 2+: _cmd.tick()
        uBit.sleep(20);
    }
}
