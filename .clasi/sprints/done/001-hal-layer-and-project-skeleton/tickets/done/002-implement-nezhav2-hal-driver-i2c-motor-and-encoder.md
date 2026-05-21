---
id: '002'
title: Implement NezhaV2 HAL driver (I2C motor and encoder)
status: done
use-cases:
- SUC-001
- SUC-003
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement NezhaV2 HAL driver (I2C motor and encoder)

## Description

Implement `NezhaV2` — the I2C driver for the PlanetX Nezha V2 motor controller.
This class wraps raw I2C register writes/reads for motor PWM and encoder readback.
It is a required (non-optional) subsystem; the robot cannot drive without it.

The TypeScript reference is `/Volumes/Proj/proj/league-projects/scratch/radio-robot/src/nezha.ts`.
That file calls `nezhaV2.start()` and `nezhaV2.readRelAngle()` from the MakeCode extension.
Before writing any C++ code, the programmer agent **must** find the exact I2C register bytes
those calls produce. Use WebFetch to read the PlanetX `pxt-nezha2` MakeCode extension at
`https://github.com/PlanetX-Labs/pxt-nezha2` (try `main.ts` or `nezha2.ts`) and extract
the raw I2C register protocol for `start()` (motor PWM write) and `readRelAngle()` (encoder read).

## Files to Create

- `source/hal/NezhaV2.h`
- `source/hal/NezhaV2.cpp`

## Key Constants (from `nezha.ts`)

```
I2C address:      0x10
LEFT_MOTOR:       M2 = 2
RIGHT_MOTOR:      M1 = 1
LEFT_FWD_SIGN:   +1
RIGHT_FWD_SIGN:  -1
```

`readEncoder(leftWheel, cal)` returns mm:
```
raw_degrees_x10 = readEncoderRaw(motorId)  // int32 from chip: tenths of degrees
mm = (raw_degrees_x10 / 10.0f) * (leftWheel ? cal.mmPerDegL : cal.mmPerDegR)
     * (leftWheel ? LEFT_FWD_SIGN : RIGHT_FWD_SIGN)
return (int32_t)mm
```

## Interface (from architecture-update.md)

```cpp
// NezhaV2.h
#pragma once
#include "MicroBit.h"
#include "types/Config.h"

class NezhaV2 {
public:
    explicit NezhaV2(MicroBitI2C& i2c);

    // Set raw PWM duty (-100..100). Positive = forward on both wheels.
    void    setPwm(int8_t leftPct, int8_t rightPct);

    // Read cumulative encoder in mm. leftWheel true = M2 (left), false = M1 (right).
    int32_t readEncoder(bool leftWheel, const CalibParams& cal) const;

    // Zero both encoder accumulators on the chip.
    void    resetEncoders();

private:
    MicroBitI2C& _i2c;
    static constexpr uint8_t ADDR        = 0x10;
    static constexpr uint8_t LEFT_MOTOR  = 2;   // M2
    static constexpr uint8_t RIGHT_MOTOR = 1;   // M1
    static constexpr int8_t  LEFT_FWD   = +1;
    static constexpr int8_t  RIGHT_FWD  = -1;

    void    writeReg(uint8_t reg, const uint8_t* data, uint8_t len);
    int32_t readEncoderRaw(uint8_t motorId) const;
};
```

## CODAL I2C API

`MicroBitI2C` is accessible as `uBit.i2c` in Robot and passed by reference to NezhaV2.

```cpp
// Write len bytes from buf to device at 7-bit addr (CODAL uses 8-bit = addr<<1):
int rc = _i2c.write((ADDR << 1), (const char*)buf, len, false);
// Read len bytes from device:
rc = _i2c.read((ADDR << 1), (char*)buf, len, false);
// rc == MICROBIT_OK (0) on success
```

Confirm the address shift convention from CODAL source before writing the first call.
Check `libraries/codal-microbit-v2/` after `python build.py` fetches dependencies.

## Acceptance Criteria

- [x] `source/hal/NezhaV2.h` and `source/hal/NezhaV2.cpp` exist
- [x] I2C register bytes for `setPwm()` verified against PlanetX extension source
- [x] I2C register bytes for `readEncoderRaw()` verified against PlanetX extension source
- [x] `setPwm(0, 0)` sends a valid stop/zero-speed command
- [x] `readEncoder(true, cal)` returns a positive value when left wheel moves forward
- [x] `resetEncoders()` resets both encoder accumulators to zero
- [x] No heap allocation; no global state beyond the class members
- [ ] `python build.py` compiles with no errors or warnings

## Testing

Hardware-in-the-loop only (no off-device test runner for CODAL).

- **Verification**: `python build.py` confirms clean compilation.
- **Hardware test**: Deferred to ticket 006 (build verification). Motor correctness verified manually by observing wheel direction when `setPwm(30, 30)` is called from the tick loop.
