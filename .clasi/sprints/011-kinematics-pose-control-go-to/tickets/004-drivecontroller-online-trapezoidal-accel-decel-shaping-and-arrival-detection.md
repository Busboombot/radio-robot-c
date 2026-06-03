---
id: '004'
title: 'DriveController: online trapezoidal accel/decel shaping and arrival detection'
status: open
use-cases:
- SUC-001
- SUC-002
depends-on:
- "011-003"
github-issue: ''
issue: kinematics-pose-control-goto.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 011-004: DriveController — online trapezoidal accel/decel shaping and arrival detection

## Description

Add the two remaining pieces that make `G` fully functional:

1. **Trapezoidal speed shaper** (kinematics-model.md §1.6): replace the
   fixed `v = _gSpeed` in the PURSUE tick with the online trapezoid — ramp
   up at `aMax`, cap by the decel curve, take the minimum.
2. **Arrival detection**: check `d_remaining < cfg.arriveTolMm` each tick;
   on arrival, call `fullStop()` and emit `EVT done G [#id]`.

After this ticket the `G` command is complete end-to-end: turn-in-place gate
(ticket 003) + pursuit steering (ticket 002) + accel/decel + arrival detection.

### New field in `DriveController.h`

```cpp
float _vRamped;   // current ramped speed, mm/s; reset to 0 at beginGoTo()
```

Add to the G go-to state block in the header.

### Changes to `DriveController.cpp`

**`beginGoTo()`** — reset `_vRamped = 0.0f` at entry (after the existing gate check
and before calling `_mc.startDriveClean()`):
```cpp
_vRamped = 0.0f;
```

**PURSUE tick — replace `float v = _gSpeed;`** with the shaper:

```cpp
float d_remaining = sqrtf(d2);   // d2 = dx*dx + dy*dy, already computed for kappa

// Accel ramp: advance _vRamped toward _gSpeed at aMax per second
float aMax   = _cfg.aMax;
float aDecel = _cfg.aDecel;
_vRamped += aMax * dt_s;
if (_vRamped > _gSpeed) _vRamped = _gSpeed;   // clamp to user max

// Decel cap: don't go faster than sqrt(2 * aDecel * d_remaining)
float v_cap = (d_remaining > 0.0f)
              ? sqrtf(2.0f * aDecel * d_remaining)
              : 0.0f;
if (v_cap < _vRamped) _vRamped = v_cap;   // clamp ramped speed to decel cap
// note: _vRamped may decrease here; it can recover on next tick if d is large

float v = _vRamped;
```

**Arrival check** — immediately after computing `d_remaining`, before computing
`kappa`:

```cpp
if (d_remaining < _cfg.arriveTolMm) {
    fullStop(dfn, dct);
    _gPhase = GPhase::IDLE;
    emitEvt("EVT done G");
    return;   // skip further PURSUE logic this tick
}
```

**`dt_s` availability in the PURSUE branch**: `dt_s` is computed at the top of
`tick()`. The PURSUE branch is inside the `if (_mode == DriveMode::GO_TO)` block,
which runs after `dt_s` is set. Confirm the variable is in scope (it is, per the
existing tick() structure); if the G block is in a nested scope, pass `dt_s` or
store it as a member for this tick.

### PRE_ROTATE speed

During PRE_ROTATE, the spin speed is set once at `beginGoTo()` and is not
subject to the accel ramp — the robot needs to turn responsively. `_vRamped`
is reset at the PRE_ROTATE → PURSUE transition (zero) so the PURSUE phase
starts its ramp fresh:

```cpp
// In PRE_ROTATE tick, on transition to PURSUE:
_vRamped = 0.0f;
_gPhase  = GPhase::PURSUE;
```

## Acceptance Criteria

- [ ] Unit test: accel ramp — at `aMax=300`, `dt=0.02s`, first tick ramps
  `_vRamped` from 0 to `min(6.0, _gSpeed)`. [unit]
- [ ] Unit test: decel cap — at `aDecel=250`, `d_remaining=10mm`,
  `v_cap = sqrt(2·250·10) = sqrt(5000) ≈ 70.7 mm/s`. [unit]
- [ ] Unit test: `v = min(v_ramped, v_cap, _gSpeed)` — all three limiting
  cases produce the correct minimum. [unit]
- [ ] Unit test: arrival gate — when `d_remaining < arriveTolMm`, returns
  (does not drive further). [unit]
- [ ] **Bench**: `G 300 0 200` — robot accelerates from rest, decelerates to
  a stop on the 300 mm mark, stops within `arriveTolMm`. [bench — HARDWARE REQUIRED]
- [ ] **Bench**: Accel phase is visually smooth (no lurch from rest); decel
  brings robot to clean stop, not coasting past the target. [bench — HARDWARE REQUIRED]
- [ ] **Bench**: `G -300 0 150` (behind) — pre-rotates, then pursues with
  smooth accel/decel, lands within `arriveTolMm`. [bench — HARDWARE REQUIRED]
- [ ] **Bench**: `EVT done G #id` is emitted exactly once on arrival and routed
  to the originating channel. [bench — HARDWARE REQUIRED]
- [ ] All existing tests pass.

## Implementation Plan

### Approach

Add `_vRamped` field; modify the PURSUE tick to replace fixed speed with the
three-way min; add the arrival check at the top of the PURSUE branch.

### Files to modify

- `source/control/DriveController.h` — add `float _vRamped`
- `source/control/DriveController.cpp` — `beginGoTo()` reset, PURSUE tick
  shaper, arrival check, PRE_ROTATE→PURSUE transition reset

### Files to create

- Unit test cases added to the test file from ticket 002 (accel ramp, decel
  cap, three-way min, arrival gate)

### Testing plan

Unit tests cover all four shaper cases (ramp-limited, cap-limited, user-max-limited,
arrival). Bench tests validate the full go-to lifecycle end-to-end from three
start positions required by the issue Verification section.

### Documentation updates

None at this stage (protocol-v2.md Named Key Table updated in ticket 005).
