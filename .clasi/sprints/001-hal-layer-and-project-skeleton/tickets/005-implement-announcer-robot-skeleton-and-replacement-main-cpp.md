---
id: '005'
title: Implement Announcer, Robot skeleton, and replacement main.cpp
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- '002'
- '003'
- '004'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement Announcer, Robot skeleton, and replacement main.cpp

## Description

This is the integration ticket. It connects all HAL drivers (tickets 002–004) into
a working Robot that boots, announces, and responds to HELLO. It also replaces the
placeholder `source/main.cpp` with a 15-line entry point.

After this ticket, the firmware should compile (`python build.py`) and pass the
SUC-001 and SUC-002 acceptance criteria on hardware.

## Files to Create

- `source/app/Announcer.h`
- `source/app/Announcer.cpp`
- `source/app/Robot.h`
- `source/app/Robot.cpp`

## Files to Modify

- `source/main.cpp` — replace the placeholder with the entry point below

---

## Announcer

Announcement format: `DEVICE:<type>:<name>:<hwName>:<serial>\n`

Field values:
- `type`   = `"Nezha2"` (compile-time constant matching `robot.platformName` in `nezha.ts`)
- `name`   = `uBit.getName()` — 5-letter codename from nRF52 FICR (e.g. "XUZIT")
- `hwName` = `"microbit"` (constant)
- `serial` = `uBit.getSerial()` — unique serial number as decimal string

The announcement string is built once in the constructor and stored in `_announcement[96]`.
`announce()` and `handle()` both call `_serial.send(_announcement)` — no re-formatting per call.

```cpp
// Announcer.h
#pragma once
#include "MicroBit.h"
#include "hal/SerialPort.h"
#include "hal/Radio.h"

class Announcer {
public:
    Announcer(MicroBit& uBit, SerialPort& serial, Radio& radio);

    // Emit the DEVICE: announcement over serial.
    void announce();

    // If line == "HELLO", re-emit announcement and return true.
    // Otherwise return false (caller processes the line normally).
    bool handle(const char* line);

private:
    SerialPort& _serial;
    Radio&      _radio;
    char        _announcement[96];
};
```

Constructor implementation sketch:
```cpp
Announcer::Announcer(MicroBit& uBit, SerialPort& serial, Radio& radio)
    : _serial(serial), _radio(radio)
{
    // Build announcement once. uBit.getName() returns a ManagedString.
    // Use snprintf with %s to assemble the announcement.
    // uBit.getSerial() returns a ManagedString of the decimal serial number.
    snprintf(_announcement, sizeof(_announcement),
             "DEVICE:Nezha2:%s:microbit:%s",
             uBit.getName().toCharArray(),
             uBit.getSerial().toCharArray());
}
```

`ManagedString::toCharArray()` returns a `const char*` valid for the duration of
the expression. Use it immediately inside `snprintf()` — do not store the pointer.

---

## Robot

Critical constraint: `MicroBit uBit` **must be the first member** of `Robot`.
C++ initializes class members in declaration order. If any driver tries to use
`uBit.i2c` before `uBit` is constructed, the behavior is undefined.

Optional sensors use `static` storage inside `Robot.cpp` to avoid heap:
```cpp
// Robot.cpp — static storage for optional sensors
static OtosSensor  s_otos(/* need i2c ref — see below */);
static LineSensor  s_line(/* ... */);
static ColorSensor s_color(/* ... */);
static GripperServo s_gripper(/* ... */);
static PortIO      s_portio(/* ... */);
```

Problem: static objects at file scope are initialized before `Robot::Robot()` runs,
which means `uBit.i2c` is not yet initialized when `s_otos(uBit.i2c)` would execute.

**Solution**: Use a static `uint8_t` buffer for each optional sensor and
placement-new inside the constructor body:
```cpp
// In Robot.h private section — raw storage, not constructed yet:
alignas(OtosSensor)  uint8_t _otosBuf[sizeof(OtosSensor)];
alignas(LineSensor)  uint8_t _lineBuf[sizeof(LineSensor)];
// etc.

// In Robot.cpp constructor body, after uBit.init():
_otos = new (_otosBuf) OtosSensor(uBit.i2c);
if (!_otos->begin()) { _otos->~OtosSensor(); _otos = nullptr; }
```

Alternatively, and more simply: declare optional sensors as non-pointer members
after `MicroBit uBit` and call `begin()` in the constructor body, storing a bool
flag for each:

```cpp
// Simpler approach — declare all as members; track presence with a bool
OtosSensor   _otosObj;
bool         _otosPresent;
// ...
// In constructor body (after uBit.init()):
_otosPresent = _otosObj.begin();
if (_otosPresent) _otosObj.init();
// Pass &_otosObj or nullptr to CommandProcessor in sprint 2.
```

Use whichever approach compiles more cleanly. The `architecture-update.md` shows
nullable pointers — either approach is acceptable as long as the tick loop
correctly skips sensors that are not present.

```cpp
// Robot.h
#pragma once
#include "MicroBit.h"
#include "types/Config.h"
#include "hal/NezhaV2.h"
#include "hal/OtosSensor.h"
#include "hal/LineSensor.h"
#include "hal/ColorSensor.h"
#include "hal/GripperServo.h"
#include "hal/PortIO.h"
#include "hal/SerialPort.h"
#include "hal/Radio.h"
#include "app/Announcer.h"

class Robot {
public:
    Robot();     // Constructs and initializes all subsystems; calls uBit.init()
    void run();  // Never returns; enters tick loop

private:
    // MUST be first — CODAL singleton
    MicroBit uBit;

    // Required subsystems (constructed from uBit references)
    NezhaV2    _motor;
    SerialPort _serial;
    Radio      _radio;
    Announcer  _announcer;
    CalibParams _cal;

    // Optional subsystems (constructed as members; _*Present tracks availability)
    OtosSensor   _otos;
    bool         _otosPresent;
    LineSensor   _line;
    bool         _linePresent;
    ColorSensor  _color;
    bool         _colorPresent;
    GripperServo _gripper;
    bool         _gripperPresent;
    PortIO       _portio;

    char _buf[128];  // shared tick-loop scratch buffer
};
```

Constructor body (Robot.cpp):
```cpp
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
    uBit.init();

    _serial.begin();
    _radio.begin();

    // Probe optional sensors
    _otosPresent = _otos.begin();
    if (_otosPresent) _otos.init();

    _linePresent  = _line.readValues(nullptr);  // or use a begin() probe if added
    _colorPresent = _color.begin();
    _gripperPresent = true;  // servo always available on P1

    _announcer.announce();
}
```

Note: `_linePresent` detection — `LineSensor` has no `begin()`; probe by attempting
a read. If the I2C returns an error code, mark as not present. Add a `bool probe()`
method to `LineSensor` if needed (write a dummy byte to 0x1A and check for ACK).

```cpp
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
```

## Replacement main.cpp

Replace `source/main.cpp` entirely with:

```cpp
#include "app/Robot.h"

static Robot robot;

int main() {
    robot.run();
    return 0;
}
```

Note: `static Robot robot` at file scope. The old `main.cpp` had a global
`MicroBit uBit` — that global **must be removed** since `MicroBit uBit` now
lives inside `Robot`. If any `source/samples/*.cpp` file references the global
`uBit`, those files may fail to compile. If that happens, the programmer agent
should either remove the failing sample files or add a `extern MicroBit uBit;`
declaration that resolves to the `Robot::uBit` member (not straightforward — it
is simpler to remove conflicting sample files from the build by deleting them).

Alternatively, keep the global `uBit` in `main.cpp` and pass it into `Robot`
by reference. Either approach is acceptable — choose whichever compiles cleanly.
Document the choice in a comment in `main.cpp`.

## Acceptance Criteria

- [x] `source/app/Announcer.h`, `Announcer.cpp`, `Robot.h`, `Robot.cpp` exist
- [x] `source/main.cpp` is the 3-line entry point (no `out_of_box_experience()` call)
- [x] `Robot` constructor calls `uBit.init()` before any subsystem initialization
- [x] `Announcer` builds the announcement once and reuses the buffer
- [x] `Announcer::handle("HELLO")` returns true and emits the announcement
- [x] `Robot::run()` calls `uBit.sleep(20)` (not busy-wait)
- [ ] `python build.py` compiles with no errors

## Testing

Hardware-in-the-loop only.

- **Boot test**: Flash and observe serial output. `DEVICE:Nezha2:` must appear
  within 3 seconds of power-on.
- **HELLO test**: Send `HELLO\n` via serial monitor; verify announcement is re-emitted.
- **Stability test**: Leave running 60 s; confirm no panic pattern on the display.
