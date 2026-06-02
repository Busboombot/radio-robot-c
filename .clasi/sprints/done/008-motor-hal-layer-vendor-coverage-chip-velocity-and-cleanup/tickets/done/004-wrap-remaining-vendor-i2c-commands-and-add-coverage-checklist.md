---
id: '004'
title: Wrap remaining vendor I2C commands and add coverage checklist
status: done
use-cases:
- SUC-004
depends-on:
- '002'
github-issue: ''
issue: nezha-full-vendor-i2c-coverage.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wrap remaining vendor I2C commands and add coverage checklist

## Description

Five Nezha2 I2C registers have no HAL wrapper in our `Motor` class:
`0x70` (timed move), `0x5D` (absolute angle), `0x1D` (reset/home),
`0x77` (global servo speed), and `0x88` (firmware version). This ticket
wraps all five and adds a coverage checklist table in `Motor.h`.

Depends on ticket 002 (Motor abstraction is complete). The vendor
reference in `vendor/pxt-nezha2/main.ts` (ticket 001) is the authoritative
frame specification.

**Critical constraint for `0x5D`**: The vendor code includes a 4 ms
post-write delay with an explicit note that no task interleave is allowed.
Resolve whether CODAL `fiber_sleep(4)` satisfies this (it yields; the I2C
hardware may be sufficient) or whether a busy-wait is needed. Document the
resolution in a code comment. Do not silently drop the delay.

These commands overlap with our own control loop for `0x70`/`0x5D` — wrap
them for completeness and diagnostic use, but do not wire them into
`DriveController`.

## Acceptance Criteria

- [x] `Motor::timedMove(uint8_t mode, int16_t value, uint8_t dir)` wraps
  `0x70`; correct frame bytes confirmed.
- [x] `Motor::moveToAngle(uint16_t angle, uint8_t mode)` wraps `0x5D`;
  4 ms post-write delay present; code comment documents task-interleave
  resolution.
- [x] `Motor::resetHome()` wraps `0x1D`.
- [x] `Motor::setGlobalSpeed(uint8_t speed)` wraps `0x77` (speed×9 →
  0–900 encoding).
- [x] `bool Motor::readVersion(uint8_t& maj, uint8_t& min, uint8_t& patch)`
  wraps `0x88`; returns true on success.
- [x] Coverage checklist table in `Motor.h` shows all 9 vendor registers
  with their HAL method names (green = wrapped, cross-ref for `0x47`).
- [x] Unit verification: for each new method, construct the expected frame
  bytes on paper (or via a host-side test) and confirm they match the
  vendor `main.ts` encoding.
- [ ] `readVersion()` returns a plausible (non-zero) version on hardware.
  (Bench validation — requires hardware)
- [x] `python3 build.py` succeeds; RAM line reported and within budget.
- [ ] Bench: `readVersion()` returns a valid version string; no I2C bus
  lockup after calling each new method once. (Bench validation — requires hardware)

## Implementation Plan

### Approach

Add five public methods to `Motor`. For each:
1. Verify the frame encoding against `vendor/pxt-nezha2/main.ts`.
2. Implement the 8-byte write (and read for `0x88`).
3. Note any timing constraints in code comments.

### Frame reference (from vendor main.ts, addr 0x10)

- `0x70` timed move: `[0xFF,0xF9,motorId, dir, 0x70, valueLow, valueHigh, mode]`
  — dir: 1=CW/2=CCW; mode: 1=turns, 2=deg, 3=sec; value is int16.
- `0x5D` abs angle: `[0xFF,0xF9,motorId, dir, 0x5D, angleLow, angleHigh, mode]`
  — BUG-critical: 4 ms post-write delay, no task interleave.
- `0x1D` reset home: `[0xFF,0xF9,motorId, 0x00, 0x1D, 0x00, 0xF5, 0x00]`
- `0x77` global speed: `[0xFF,0xF9,motorId, 0x00, 0x77, speedEncLow, speedEncHigh, 0x00]`
  where `speedEnc = speed * 9` (0–900).
- `0x88` read version: write `[0xFF,0xF9,0x00,0x00,0x88,0x00,0xF5,0x00]`;
  read 3 bytes: major, minor, patch.

Verify each frame byte-for-byte against `vendor/pxt-nezha2/main.ts` before
implementing.

### Files to Modify

- `source/hal/Motor.h` — declare 5 new methods + coverage checklist table
- `source/hal/Motor.cpp` — implement 5 new methods

### Testing Plan

- For each new method: write expected frame bytes as a comment in the
  implementation; reviewer verifies against vendor TS.
- Bench: call each method once and observe no I2C lockup; `readVersion()`
  returns non-zero bytes.
- `python3 build.py` must succeed; report RAM line.

### Documentation Updates

- `Motor.h` coverage checklist: all 9 registers (including `0x46`, `0x47`,
  `0x60`, `0x5F` already wrapped) shown with method names.
