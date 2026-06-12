---
id: '006'
title: 'N6: Extend validateConfig() to cover rate/accel/timeout family'
status: open
use-cases:
  - SUC-005
depends-on: []
github-issue: ''
issue: fr2-n6-config-validation.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# N6: Extend validateConfig() to cover rate/accel/timeout family

## Description

`validateConfig()` (`ConfigRegistry.cpp:239-258`) currently checks only: tw,
ctrlPeriod, vWheelMax/steerHeadroom, rotSlip. Several foot-gun values are unchecked:

- `SET aDecel=-100`: trapezoid `dv_max` goes negative; `approach()` moves away from
  the target each tick (`BodyVelocityController.cpp:84-85`) — runaway. Decel caps
  compute `sqrtf(negative)` — NaN comparisons silently disable them.
- `SET aMax=0` / `yawAccMax=0`: BVC can never leave zero; every motion verb stalls
  to its TIME net (robot looks dead).
- `SET sTimeout=0` (or negative): watchdog compare fires every tick once armed — X
  storm (continuous emergency stops).
- `SET vBodyMax=0`, `yawRateMax=0`: all motion targets clamp to zero.

Also: `effectiveSlip()` accepts `rotSlip=0` as "unset → 1.0" but `validateConfig`
rejects it, so a host cannot restore the documented "unset" state.

## Acceptance Criteria

- [ ] `validateConfig()` rejects `aDecel <= 0`, `aMax <= 0`, `vBodyMax <= 0`,
      `yawRateMax <= 0`, `yawAccMax <= 0` (each returns `ERR badval`).
- [ ] `validateConfig()` rejects `sTimeoutMs` below a reasonable floor (e.g. <= 0,
      or <= 100 ms — choose a value that prevents the X-storm without breaking
      legitimate short timeouts).
- [ ] `rotSlip=0` is accepted by `validateConfig()` (treated as "unset → 1.0"
      matching `effectiveSlip()` semantics).
- [ ] New sim tests: each of `SET aDecel=-100`, `SET aMax=0`, `SET sTimeout=0`,
      `SET vBodyMax=0`, `SET yawRateMax=0` returns `ERR badval` and leaves live
      config unchanged.
- [ ] Existing config validation tests pass without regression.
- [ ] `python3 build.py` clean build passes.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.

## Implementation Plan

### Approach

Extend the validation block in `validateConfig()` with `> 0` checks for the
rate/accel family and a floor check for `sTimeoutMs`. Fix the `rotSlip` inconsistency
by changing the `rotSlip` check from `<= 0` to `< 0`.

### Files to modify

- `source/config/ConfigRegistry.cpp`
  - `validateConfig()` (:239-258): add checks for `aMax`, `aDecel`, `vBodyMax`,
    `yawRateMax`, `yawAccMax` (reject if <= 0); add floor check for `sTimeoutMs`
    (reject if <= 0 or below chosen floor); change `rotSlip` check from `<= 0`
    to `< 0`.
- `host_tests/` or `host/tests/` — add parameterized test covering each new
  rejection case and the `rotSlip=0` acceptance case.

### Testing plan

Run: `uv run --with pytest python -m pytest host_tests/ host/tests/ -v`

Build: `python3 build.py` (clean).

### Notes

- Independent of tickets 001-005 (only `ConfigRegistry.cpp` changes).
- DefaultConfig.cpp is auto-generated from `tovez.json`. If any default config
  values change (none expected for this ticket — we're only adding validation),
  regenerate via `scripts/gen_default_config.py`; never hand-edit DefaultConfig.cpp.
- The `sTimeoutMs` floor value should be documented in a comment in validateConfig().
