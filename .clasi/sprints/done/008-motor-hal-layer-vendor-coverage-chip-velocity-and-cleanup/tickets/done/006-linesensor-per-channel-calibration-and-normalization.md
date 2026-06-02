---
id: '006'
title: LineSensor per-channel calibration and normalization
status: done
use-cases:
- SUC-006
depends-on:
- '002'
github-issue: ''
issue: source-fixme-cleanup.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# LineSensor per-channel calibration and normalization

## Description

`LineSensor` currently returns raw 0–255 grayscale values with no
normalization. The FIXME asked for a calibration flow where the user sweeps
the robot to capture per-sensor min/max, then scales each channel to a
normalized response robust to lighting variation, with optional EMA
smoothing.

Depends on ticket 002 because `Robot.h` will have been restructured; this
ticket adds to `LineSensor` without touching the Motor path and should apply
cleanly on top.

**RAM budget note**: The calibration arrays add 4×2×2 = 16 bytes. EMA state
adds 4 floats = 16 bytes. Total: ~32 bytes. If RAM is tight after tickets
002–005, omit EMA smoothing and defer it to a future sprint.

## Acceptance Criteria

- [x] `LineSensor` stores `uint16_t _calMin[4]` and `_calMax[4]`
  (initialized to 0/255).
- [x] `captureCalibMin()` snapshots current raw readings into `_calMin`.
- [x] `captureCalibMax()` snapshots current raw readings into `_calMax`.
- [x] `readNormalized(uint16_t out[4])` returns 0–1000 per channel:
  `(raw - min) * 1000 / (max - min)`, clamped to [0, 1000].
- [x] Existing `readValues(uint16_t out[4])` unchanged (raw reads still
  available).
- [x] `float _alpha` EMA smoothing coefficient (0.0 = no smoothing);
  configurable via `setSmoothingAlpha(float alpha)`. Applied in
  `readNormalized` only. (Defer if RAM is over budget.)
- [x] `python3 build.py` succeeds; RAM line reported. If RAM exceeds budget,
  omit EMA (remove `_alpha` and `setSmoothingAlpha`) and note the deferral.
- [ ] Bench: place robot over white surface → `captureCalibMin()`; place
  over black → `captureCalibMax()`; confirm `readNormalized()` returns ~0
  over white and ~1000 over black on all 4 channels.

## Implementation Plan

### Approach

Additive changes to `LineSensor` only — no other files need touching.

1. Add `uint16_t _calMin[4]`, `_calMax[4]` initialized to {0,0,0,0} /
   {255,255,255,255} (safe defaults: if never calibrated, output is a
   reasonable range).
2. Add `captureCalibMin()` and `captureCalibMax()`: call `readValues` and
   copy results into the calibration arrays.
3. Add `readNormalized(uint16_t out[4])`: call `readValues`, then for each
   channel: `span = max - min` (if span == 0, use 255); `norm = (raw - min)
   * 1000 / span`; clamp to [0, 1000].
4. Add `float _alpha` (default 0.0f) and `float _emaState[4]` (default
   0.0f); apply EMA in `readNormalized` if `_alpha > 0.0f`.
5. Add `setSmoothingAlpha(float)` setter.

### Files to Modify

- `source/hal/LineSensor.h` — new members + method declarations
- `source/hal/LineSensor.cpp` — implement new methods

### Testing Plan

- `python3 build.py` must succeed; report RAM line.
- Bench calibration procedure:
  1. Place on white surface; call `captureCalibMin()`.
  2. Place on black surface; call `captureCalibMax()`.
  3. Read `readNormalized()` on white → all channels near 0.
  4. Read `readNormalized()` on black → all channels near 1000.
  5. Move across line boundary → channels crossing the line transition
     smoothly 0→1000.

### Documentation Updates

- `LineSensor.h` header comment: document calibration workflow.
- `docs/architecture.md`: note calibration capability in LineSensor
  description.
