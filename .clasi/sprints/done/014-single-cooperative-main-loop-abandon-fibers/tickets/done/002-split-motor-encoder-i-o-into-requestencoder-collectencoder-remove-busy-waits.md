---
id: '002'
title: Split Motor encoder I/O into requestEncoder / collectEncoder (remove busy-waits)
status: done
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: plan-single-cooperative-main-loop-abandon-fibers.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Split Motor encoder I/O into requestEncoder / collectEncoder (remove busy-waits)

## Description

Replace `Motor::readEncoderRaw()` (which contains two 4 ms busy-wait loops
and performs a synchronous write+read) with two non-blocking methods:

- `requestEncoder()` — sends the `0x46` command write and returns immediately.
  No wait.
- `collectEncoder()` — reads back the 4-byte response and returns the raw
  int32 (tenths of degrees, minus software offset). No wait. The caller must
  ensure at least one control period has elapsed since `requestEncoder()` —
  the loop's idle sleep provides this guarantee.

The existing `readEncoderMmF()` and `readEncoder()` public methods are updated
to call `collectEncoder()` internally (with the split-phase contract documented
clearly). `readSpeedRaw()` (0x47, already disabled at the `MotorController`
level) is left intact and unchanged.

## Files to Modify

- `source/hal/Motor.h` — add `requestEncoder()` and `collectEncoder()` public
  declarations; mark or remove `readEncoderRaw()` private.
- `source/hal/Motor.cpp` — implement `requestEncoder()` and `collectEncoder()`;
  delete both busy-wait spin loops from `readEncoderRaw()`.

## Acceptance Criteria

- [x] `Motor::requestEncoder()` issues the `0x46` write and returns without any
  busy-wait or `fiber_sleep`.
- [x] `Motor::collectEncoder()` issues the 4-byte read and returns the signed
  int32 (raw tenths of degrees minus `_encOffset`), without any busy-wait or
  `fiber_sleep`.
- [x] Both `system_timer_current_time_us` busy-wait spin loops in
  `readEncoderRaw()` are deleted.
- [x] `readEncoderMmF()` and `readEncoder()` are updated to call
  `collectEncoder()` with a comment noting the split-phase contract.
- [x] Firmware builds cleanly with no new errors. (At this stage
  `MotorController` still calls the old encoder path — the build must not
  break between tickets.)

## Implementation Plan

1. In `Motor.h`, add to the public interface:
   - `void    requestEncoder()` — fire the 0x46 write; return immediately.
   - `int32_t collectEncoder()` — read back 4-byte response; return raw int32
     minus `_encOffset`. Document the split-phase contract.
2. In `Motor.cpp`, implement `requestEncoder()`:
   - Copy the 8-byte `0x46` command buffer from `readEncoderRaw()`.
   - Call `_i2c.write()`; return. No waits.
3. In `Motor.cpp`, implement `collectEncoder()`:
   - Call `_i2c.read()` for 4 bytes.
   - Reconstruct the little-endian int32.
   - Subtract `_encOffset`. Return.
4. In `readEncoderRaw()`: delete both `system_timer_current_time_us`
   while-loops. Keep the write and read calls so the method still works
   (without delay) during the transition window.
5. Update `readEncoderMmF()` and `readEncoder()` to call `collectEncoder()`
   instead of `readEncoderRaw()`.

## Testing Plan

- **Build verification** (CI): `python build.py` — no new errors.
- **Automated tests**: `uv run --with pytest python -m pytest` — all tests must
  pass. The pytest suite mocks I2C at the protocol layer; removal of busy-waits
  is transparent to test fixtures.
- **Hardware bench**: Deferred to ticket 009 (final bench gate). The
  non-blocking encoder path is exercised end-to-end only once `LoopScheduler`
  is running.
