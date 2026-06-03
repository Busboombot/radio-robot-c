---
id: '004'
title: Fix chip readSpeed I2C context + encoder-delta fallback plausibility gate
status: done
use-cases:
- SUC-002
- SUC-007
depends-on: []
github-issue: ''
issue: ''
completes_issue: false
---

# Fix chip readSpeed I2C context + encoder-delta fallback plausibility gate

## Description

**Critical**: The Nezha V2 chip register 0x47 (`readSpeed`) is NOT broken — the
vendor MakeCode `nezhaV2.readSpeed(M1)` returns sensible, increasing values when
run in isolation (start -> pause 500 ms -> readSpeed). But our C++ returns a
stuck ~30-33 mm/s at all commanded speeds.

Root cause: tight-loop I2C interleaving. `MotorController::tick()` hammers the
Nezha (address 0x10) every 20 ms with: 0x46 enc read x2, 0x47 speed read x2,
0x60 write x2, plus OTOS/line/color on the same bus. Register 0x47 is read in
a bad window before the chip's internal speed estimate has settled after a motor
command write.

**Do NOT demote the chip to encoder-delta-only.** The chip works. Fix the read
context so it tracks actual speed.

**Reference oracle**: `vendor/pxt-nezha2/main.ts` readSpeed function. Use this
as the ground truth for what timing/sequence produces a good read.

The existing plausibility gate rejects only too-high chip readings. Improve it
to also reject stuck/too-low readings (both sides).

## Files to Modify

- **`source/hal/Motor.cpp`** — `readSpeedRaw()`:
  - Investigate and fix the settle delay or ordering issue. Options to explore
    (in order of preference):
    1. Increase the post-write settle delay from 4 ms to a larger value (e.g. 8 ms
       or match the vendor's 500 ms pause in isolation).
    2. Separate the speed read from the encoder read so they are not interleaved
       in the same tick (read speed less frequently, e.g. every other tick).
    3. Read speed immediately before issuing the next motor command (so the chip
       has had the full inter-tick interval to settle).
  - Also resolve the `kUnitFactor` question: is 0x47 raw in tenths-of-deg/s
    (current: `kUnitFactor=10.0`) or whole-deg/s (`kUnitFactor=1.0`)? Compare
    `readSpeed()` output to encoder-delta at a known steady speed. Set the
    correct value and remove the `BENCH-CONFIRM` comment.

- **`source/control/MotorController.cpp`** — tick() plausibility gate:
  Update the gate to reject BOTH too-high and too-low chip readings:
  ```cpp
  // Reject if chip reads > 2x encoder (too-high / noise)
  // Also reject if chip reads < 0.5x encoder when wheel clearly moving (stuck / too-low)
  bool tooHigh = fabsf(chipVelL) > 2.0f * fabsf(encVelL);
  bool tooLow  = (fabsf(encVelL) > _cal.minWheelMms) &&
                 (fabsf(chipVelL) < 0.5f * fabsf(encVelL));
  if (chipOkL && (fabsf(encVelL) > 0.0f) && (tooHigh || tooLow)) chipOkL = false;
  ```
  Apply same pattern for chipOkR.

- **`source/hal/Motor.h`** — no interface change.
- **`tests/test_readspeed_and_get_vel.py`** — update to reflect the corrected
  behavior: `GET VEL` source flag 'C' at non-zero speed; encoder fallback 'E'
  only on simulated failure.

## Approach

1. Read `vendor/pxt-nezha2/main.ts` readSpeed to understand the vendor's timing.
2. On hardware, run a diagnostic: log `readSpeedRaw()` raw values alongside
   encoder-delta at several commanded speeds. Identify the stuck pattern.
3. Try fix options in order. After each, run `S 200 200` and observe `GET VEL`.
   The chip source 'C' should scale with the command (e.g. ~200 mm/s at `S 200`).
4. Confirm the `kUnitFactor` (tenths or whole deg/s) by comparing readSpeed output
   to encoder-delta at a steady speed. Correct the constant and remove the comment.
5. Update the plausibility gate.
6. Update `test_readspeed_and_get_vel.py`.
7. Clean build. Reflash robot enum 2. Bench verification.

## Acceptance Criteria

- [ ] `GET VEL` at commanded `S 100 100` returns source 'C' with values approximately 100 mm/s (not stuck at ~30-33). [BENCH DEFERRED — T11]
- [ ] `GET VEL` at commanded `S 300 300` returns source 'C' with values approximately 300 mm/s. [BENCH DEFERRED — T11]
- [ ] At idle (motor stopped), `GET VEL` returns approximately 0 mm/s. [BENCH DEFERRED — T11]
- [ ] `kUnitFactor` is correct (determined by hardware observation); `BENCH-CONFIRM` comment is removed. [BENCH DEFERRED — T11]
- [x] Plausibility gate rejects both too-high AND too-low chip readings.
- [x] Encoder-delta fallback ('E') activates only on genuine I2C failure, not on normal operation.
- [x] `tests/test_readspeed_and_get_vel.py` updated and passing.
- [x] Clean build (`mbdeploy build --clean`) succeeds.
- [ ] (Bench deferred to T11) Straight-line drive with chip velocity PID shows reduced lateral drift.

## Testing

- **Existing tests to update**: `tests/test_readspeed_and_get_vel.py`
- **Hardware diagnostic**: log raw `readSpeedRaw()` output at multiple commanded speeds before fixing.
- **Verification command**: `mbdeploy build --clean && uv run pytest tests/test_readspeed_and_get_vel.py`
