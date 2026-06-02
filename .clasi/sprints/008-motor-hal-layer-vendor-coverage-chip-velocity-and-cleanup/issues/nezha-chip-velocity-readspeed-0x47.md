---
status: in-progress
sprint: 008
tickets:
- 008-003
---

# Nezha Chip-Native Wheel Velocity via readSpeed (0x47)

**Priority issue.** Split from the Nezha vendor-I2C-coverage plan; sibling of
[[nezha-full-vendor-i2c-coverage]].

## Context

The Nezha2 motor controller reports wheel **velocity** on-board via I2C register
`0x47` (`readSpeed` in the advisory vendor driver `pxt-nezha2/main.ts`). Our C++
HAL does not expose it — `0x47` is absent in `source/hal/NezhaV2.cpp` (it was
only ever a `return 0` stub in the old TypeScript). Today `MotorController`
derives velocity by differencing encoder ticks over dt
(`source/control/MotorController.cpp:112-113`). If the chip measures velocity
directly we want that as the source of truth, with tick-differencing kept as a
validated fallback.

This de-risks the future velocity-PID / controller rewrite; it stands alone and
is the priority item.

## Vendor reference

8-byte frame `[0xFF,0xF9,motor,p3,reg,p5,p6,p7]`, chip addr `0x10`,
**M1 = right, M2 = left**. `0x47`: write request, read **uint16 LE**, convert
`floor(raw/3.6)*0.01` → **laps/s**. Encoder/speed reads use **4 ms pre-write and
4 ms post-write delays** (`fiber_sleep(4)` — mirror `readEncoderRaw`).
Authoritative source:
`/Volumes/Proj/proj/league-projects/scratch/radio-robot/vendor/pxt-nezha2/main.ts`.

## Scope

**HAL (`source/hal/NezhaV2.{h,cpp}`):**
- Private `int16_t readSpeedRaw(motorId)` issuing
  `[0xFF,0xF9,motor,0x00,0x47,0x00,0xF5,0x00]`, 2-byte LE read, 4 ms pre/post
  delays.
- Public `bool readSpeed(bool leftWheel, float& mmPerSec)`: `floor(raw/3.6)*0.01`
  → laps/s, apply per-wheel forward sign (mirror `LEFT_FWD`/`RIGHT_FWD` in
  `setPwm`), then laps/s → mm/s. **Do not assume the laps→mm scale** — the
  encoder's `mmPerDeg` calibration may define a different "lap" than the chip;
  pin it empirically (below). Geometry/calibration constants live in
  `source/types/Config.h`.

**Wiring (`source/control/MotorController.{h,cpp}`, `Odometry`):**
- Velocity source: **chip velocity primary**, encoder-tick differentiation
  fallback. Fallback triggers on I2C error or implausible reading. Expose which
  source is live (telemetry/debug).

## Empirical validation (bench — required before trusting it)

Drive at several known PWMs; log `(pwm, chip mm/s, encoder-derived mm/s)`.
Confirm monotonicity, sign agreement, latency, quantization (~2.54 mm/s/LSB
expected → low-speed ripple). Pin the laps→mm/s scale from this data. If
unreliable, fall back to encoder-derived velocity — a **documented** outcome, not
a silent failure.

## Verification

- Unit-test the `0x47` frame bytes and the raw→laps→mm/s conversion.
- Bench log/plot chip vs encoder-derived velocity across PWMs (monotonic, correct
  sign, acceptable latency); laps→mm/s scale pinned.
- Exercise the fallback by simulating an I2C error.
