---
id: '004'
title: Implement sensor HAL drivers (OtosSensor, LineSensor, ColorSensor, GripperServo,
  PortIO)
status: done
use-cases:
- SUC-001
- SUC-003
depends-on:
- '001'
- '002'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement sensor HAL drivers (OtosSensor, LineSensor, ColorSensor, GripperServo, PortIO)

## Description

Implement the five optional peripheral HAL drivers. All five are stored as
nullable pointers in `Robot` — the robot boots and runs without any of them
connected. The dependency on ticket 002 (`NezhaV2`) is for pattern reference
only (same I2C helper pattern); these drivers do not call `NezhaV2`.

Reference files for register maps and init sequences:
- OTOS: `/Volumes/Proj/proj/league-projects/scratch/radio-robot/src/otos.ts`
- LineSensor, ColorSensor, GripperServo, PortIO: `/Volumes/Proj/proj/league-projects/scratch/radio-robot/src/nezha.ts`

## Files to Create

- `source/hal/OtosSensor.h` and `source/hal/OtosSensor.cpp`
- `source/hal/LineSensor.h` and `source/hal/LineSensor.cpp`
- `source/hal/ColorSensor.h` and `source/hal/ColorSensor.cpp`
- `source/hal/GripperServo.h` and `source/hal/GripperServo.cpp`
- `source/hal/PortIO.h` and `source/hal/PortIO.cpp`

---

## OtosSensor

Register map (from `otos.ts` — all multi-byte registers are little-endian signed int16):

```
0x00  PRODUCT_ID      (read; expected 0x5F)
0x01  HW_VERSION
0x02  FW_VERSION
0x04  LINEAR_SCALAR   (signed int8, 0.1% resolution; 0 = 1.000x)
0x05  ANGULAR_SCALAR  (signed int8, 0.1% resolution)
0x06  IMU_CALIBRATION (write N samples to start; reads back as it decrements)
0x07  RESET           (bit 0: reset Kalman tracking)
0x0E  SIGNAL_PROCESS_CFG  (bits: LUT=0x01, Accel=0x02, Rotation=0x04, Variance=0x08)
0x0F  SELF_TEST
0x10  OFFSET_XL       (6 bytes: X_L X_H Y_L Y_H H_L H_H, signed int16 LE)
0x1F  STATUS
0x20  POSITION_XL     (6 bytes, same format)
0x26  VELOCITY_XL     (6 bytes)
0x2C  ACCELERATION_XL (6 bytes)
```

LSB conversions: 1 position LSB ≈ 0.305 mm; 1 heading LSB ≈ 0.00549°.

Interface:
```cpp
#pragma once
#include "MicroBit.h"
#include <stdint.h>

class OtosSensor {
public:
    explicit OtosSensor(MicroBitI2C& i2c);

    // Returns false if PRODUCT_ID != 0x5F (sensor not connected or wrong device).
    bool begin();

    // Enable all signal processing (0x0F) and reset Kalman tracking.
    void init();

    // Write N to REG_IMU_CALIBRATION. Calibration runs asynchronously.
    void calibrateImu(uint8_t samples);

    // Write 0x01 to REG_RESET (resets Kalman filters, not position).
    void resetTracking();

    void getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const;
    void setPositionRaw(int16_t x, int16_t y, int16_t h);
    void getVelocityRaw(int16_t& x, int16_t& y, int16_t& h) const;

    int8_t getLinearScalar() const;
    void   setLinearScalar(int8_t val);
    int8_t getAngularScalar() const;
    void   setAngularScalar(int8_t val);

private:
    MicroBitI2C& _i2c;
    static constexpr uint8_t ADDR = 0x17;

    void    writeReg8(uint8_t reg, uint8_t val);
    uint8_t readReg8(uint8_t reg) const;
    // Burst read 6 bytes from a triple-register (X_L X_H Y_L Y_H H_L H_H).
    void    readXYH(uint8_t startReg, int16_t& x, int16_t& y, int16_t& h) const;
    // Burst write three signed int16 to a triple-register block.
    void    writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h);
};
```

---

## LineSensor

Protocol (from `nezha.ts` `readLineGrays()`): write 1-byte channel index (0–3)
to address 0x1A, then read 1 byte: grayscale 0–255.

```cpp
#pragma once
#include "MicroBit.h"
#include <stdint.h>

class LineSensor {
public:
    explicit LineSensor(MicroBitI2C& i2c);
    // Fills out[0..3] with grayscale values (0=white, 255=black approx).
    // Returns false on I2C error.
    bool readValues(uint16_t out[4]) const;
private:
    MicroBitI2C& _i2c;
    static constexpr uint8_t ADDR = 0x1A;
};
```

Implementation: loop channels 0–3; for each: write the channel byte, read 1 byte.
Use the same `writeReg8` / `readReg8` helpers as OtosSensor.

---

## ColorSensor

Two-chip detection (from `nezha.ts` `initColor()`):

Alt chip at 0x43:
- Write reg 0x81 = 0xCA, reg 0x80 = 0x17
- Read `c_probe = readReg16(0xA4)` (two bytes, lo+hi×256)
- If `c_probe != 0` → alt chip confirmed; use 0x43

APDS9960 at 0x39 (fallback):
```
Write 0x81 = 252  (ATIME)
Write 0x8F = 0x03 (CONTROL: 4× gain)
Write 0x80 = 0x00 (ENABLE: power off)
Write 0xAB = 0x00
Write 0xE7 = 0x00
Write 0x80 = 0x01 (ENABLE: power on)
Read 0x80, OR with 0x02, write back (enable AEN bit)
```

Alt chip RGBC read (blocking: pause 100 ms then read):
```
C: readReg16(0xA6)
R: readReg16(0xA0)
G: readReg16(0xA2)
B: readReg16(0xA4)
```

APDS9960 RGBC read (poll STATUS bit 0 = AVALID, max 50 tries × 5 ms):
```
while ((readReg8(0x93) & 0x01) == 0 && tries < 50) { sleep(5); tries++; }
C: readReg16(0x94)
R: readReg16(0x96)
G: readReg16(0x98)
B: readReg16(0x9A)
```

```cpp
#pragma once
#include "MicroBit.h"
#include <stdint.h>

class ColorSensor {
public:
    explicit ColorSensor(MicroBitI2C& i2c);

    // Auto-detect chip variant. Returns false if neither 0x43 nor 0x39 responds.
    bool begin();

    // Fills r,g,b,c with 16-bit raw counts. Blocks ≤250 ms.
    bool readRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c);

private:
    MicroBitI2C& _i2c;
    bool _isAlt;
    bool _inited;

    static constexpr uint8_t ADDR_APDS = 0x39;
    static constexpr uint8_t ADDR_ALT  = 0x43;

    void    initApds();
    void    initAlt();
    void    writeReg8(uint8_t addr, uint8_t reg, uint8_t val);
    uint8_t readReg8(uint8_t addr, uint8_t reg) const;
    uint16_t readReg16(uint8_t addr, uint8_t regLo) const;  // reads loReg, hiReg=loReg+1
};
```

Note: Because `ColorSensor` uses two different I2C addresses, the address
must be passed to every `writeReg8`/`readReg8` call rather than stored as a constant.

---

## GripperServo

CODAL pin API: `MicroBitPin::setServoValue(int degrees)` — range 0–180.
Before implementing, verify this method exists on `MicroBitPin` by searching
`libraries/codal-microbit-v2/` after the first build fetches dependencies.
Alternative if not found: `setAnalogValue()` with PWM period set for 50 Hz.

```cpp
#pragma once
#include "MicroBit.h"
#include <stdint.h>

class GripperServo {
public:
    explicit GripperServo(MicroBitPin& pin);  // pass uBit.io.P1
    // Set servo angle. Clamps to 0..180.
    void setAngle(uint8_t degrees);
private:
    MicroBitPin& _pin;
};
```

---

## PortIO

Pin mapping (from `nezha.ts`):
- Digital S2: J1→P8, J2→P12, J3→P14, J4→P16
- Analog  S1: J1→P1,  J2→P2,  J3→P13, J4→P15

```cpp
#pragma once
#include "MicroBit.h"
#include <stdint.h>

class PortIO {
public:
    explicit PortIO(MicroBitIO& io);  // pass uBit.io
    void setDigital(uint8_t port, bool high);   // port 1..4; ignores out-of-range
    int  readDigital(uint8_t port) const;       // returns 0, 1, or -1
    void setAnalog(uint8_t port, uint16_t val); // 0..1023
    int  readAnalog(uint8_t port) const;        // returns 0..1023 or -1
private:
    MicroBitIO& _io;
    MicroBitPin* digitalPin(uint8_t port) const;
    MicroBitPin* analogPin(uint8_t port) const;
};
```

CODAL IO API:
- `uBit.io.P8.setDigitalValue(1)` / `getDigitalValue()` for digital
- `uBit.io.P1.setAnalogValue(val)` / `getAnalogValue()` for analog (0–1023)

`digitalPin()` returns `&_io.P8`, `&_io.P12`, `&_io.P14`, or `&_io.P16`.
`analogPin()` returns `&_io.P1`, `&_io.P2`, `&_io.P13`, or `&_io.P15`.

---

## Acceptance Criteria

- [x] All five `.h` and `.cpp` files exist in `source/hal/`
- [x] `OtosSensor::begin()` returns false when the device is not connected (PRODUCT_ID check)
- [x] `OtosSensor::readXYH()` performs a 6-byte burst I2C read starting at the given register
- [x] `LineSensor::readValues()` reads 4 channels using the write-then-read protocol
- [x] `ColorSensor::begin()` probes 0x43 first, falls back to 0x39
- [x] `ColorSensor::readRGBC()` uses the correct registers for the detected chip variant
- [x] `GripperServo::setAngle()` clamps input to 0..180 before calling CODAL pin API
- [x] `PortIO::readDigital()` and `readAnalog()` return -1 for port values outside 1..4
- [x] No heap allocation in any driver
- [ ] `python build.py` compiles with no errors or warnings

## Testing

Hardware-in-the-loop only.

- **Verification**: `python build.py` confirms clean compilation. Functional
  testing of individual sensors deferred to sprint 3 (full sensor command set).
- **Boot test** (ticket 006): Robot constructs all optional sensors with null-checks;
  the absence of a sensor does not cause a panic.
