---
id: "004"
title: "Migrate D distance-drive onto MotionCommand DISTANCE stop with terminal decel"
status: open
use-cases:
- SUC-004
depends-on:
- "003"
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migrate D distance-drive onto MotionCommand DISTANCE stop with terminal decel

## Description

Replace `beginDistance`'s bespoke encoder-delta branch with a MotionCommand having a
`DISTANCE(mm)` stop condition plus a safety-net `TIME` stop (the existing timeout
heuristic). Add a per-tick terminal decel cap hook analogous to G's pursuit hook.
Preserve the encoder-reset workaround. `EVT done D` wire contract preserved.

**Key changes in `beginDistance`:**
1. Call `BodyKinematics::forward(L, R, trackwidthMm, v, omega)`.
2. Call `_mc.resetEncoderAccumulators()` (encoder-reset workaround — keep this).
3. Configure `_activeCmd` with target `(v, omega)`:
   - `addStop(makeDistanceStop(targetMm))` — primary stop.
   - Compute timeout: same formula as before (`2× nominal + 2 s`); `addStop(makeTimeStop(timeoutMs))`.
   - `setDoneEvt("EVT done D")`, SOFT style.
   - Capture reply sink + corr_id. Call `_activeCmd.start(*_hwState, now_ms)`.
4. Store `_dDistTarget = targetMm`, `_dOmega = omega` for the per-tick decel hook.
5. Remove old `startDriveClean`, `setTarget`, `_dEncStartL/R`, `_dTargetMm`,
   `_dTimeoutMs` assignments.

**Per-tick D decel hook in `driveAdvance`:**
When `_activeCmd.active() && _mode == DriveMode::DISTANCE`:
- Compute `enc_avg = (inputs.encLMm + inputs.encRMm) * 0.5f`.
- `d_traveled = fabsf(enc_avg - _activeCmd baseline enc0)` — but note: baseline is
  captured inside MotionCommand and not directly accessible. Alternative: store
  `_dEnc0` at begin to mirror the baseline.
- `d_remaining = _dDistTarget - d_traveled`.
- If `d_remaining > 0`: `v_cap = sqrtf(2.0f * _cfg.aDecel * d_remaining)`.
- Compute current `v` from BVC: `_bvc.currentV()`.
- If `v_cap < _bvc.targetV()`: call `_activeCmd.setTarget(v_cap, _dOmega)`.
  (Only clamp downward; do not increase speed beyond what was commanded.)

Store `_dEnc0` (float, enc average at begin) in DriveController to support the
decel cap. This replaces `_dEncStartL` + `_dEncStartR`.

**Member changes:**
- Remove: `_dEncStartL`, `_dEncStartR`, `_dTargetMm`, `_dTimeoutMs`.
- Add: `float _dDistTarget`, `float _dOmega`, `float _dEnc0`.

**EVT done D:** grep all tests and calibration scripts for `done D` before modifying
any emission path.

## Acceptance Criteria

- [ ] `_dEncStartL/R`, `_dTargetMm`, `_dTimeoutMs` members removed (grep confirms).
- [ ] `_dDistTarget`, `_dOmega`, `_dEnc0` added.
- [ ] `EVT done D` wire format preserved (grep `done D` in all test files before editing).
- [ ] `DISTANCE` stop uses encoder sum from HardwareState (`encLMm + encRMm)/2`
  consistent with `StopCondition::evaluate` — no filtered-value stall.
- [ ] Encoder-reset workaround (`resetEncoderAccumulators()`) still called before
  `_activeCmd.start()`.
- [ ] D-timeout heuristic retained: `2× nominal + 2 s` TIME stop alongside DISTANCE stop.
- [ ] Terminal decel cap applied per tick; robot decelerates smoothly near target.
- [ ] D branch removed from `driveAdvance`.
- [ ] `uv run --with pytest python -m pytest -q` passes at 1179/8 baseline.
- [ ] Clean build: `python3 build.py --clean` succeeds.

## Implementation Plan

### Files to modify
- `source/control/DriveController.h`:
  - Remove: `int32_t _dEncStartL`, `_dEncStartR`, `_dTargetMm`; `uint32_t _dTimeoutMs`
  - Add: `float _dDistTarget`, `float _dOmega`, `float _dEnc0`
- `source/control/DriveController.cpp`:
  - Constructor: update initialisers
  - `beginDistance`: rewrite per description above
  - `driveAdvance`:
    - Remove the `if (_mode == DISTANCE)` block
    - In the `_activeCmd.active()` early-return block, add D decel hook before `_activeCmd.tick()`
    - Hook structure mirrors G pursuit hook pattern from ticket 002

### D-decel hook control flow (mirrors G)
```cpp
if (_activeCmd.active()) {
    if (_mode == DriveMode::DISTANCE) {
        float enc_avg = (inputs.encLMm + inputs.encRMm) * 0.5f;
        float d_traveled = fabsf(enc_avg - _dEnc0);
        float d_remaining = _dDistTarget - d_traveled;
        if (d_remaining > 0.0f) {
            float v_cap = sqrtf(2.0f * _cfg.aDecel * d_remaining);
            if (v_cap < _bvc.targetV()) {
                _activeCmd.setTarget(v_cap, _dOmega);
            }
        }
    }
    bool running = _activeCmd.tick(inputs, now_ms, dt_s);
    if (!running) { _mode = DriveMode::IDLE; target.mode = DriveMode::IDLE; }
    return;
}
```

### Testing plan
- Grep `done D` in all test and calibration files.
- Run `tests/dev/test_motion_verbs_v2.py` and any D-specific tests.
- Full pytest suite: `uv run --with pytest python -m pytest -q`.
- Verify timeout formula: for `D 200 200 400`, nominal = 400/200 * 1000 = 2000 ms;
  timeout = 2*2000 + 2000 = 6000 ms — still well above actual time with ramp-up.
- Bench (stakeholder-deferred): `D 200 200 400` terminates accurately at 400 mm; no spasm.
