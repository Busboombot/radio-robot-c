---
id: '006'
title: Apply rotational-slip correction in Odometry predict and RT wheel-arc target
status: open
use-cases:
  - SUC-002
  - SUC-004
  - SUC-005
depends-on:
  - '004'
  - '005'
github-issue: ''
issue: d02-apply-rotational-slip.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 024-006 — Apply rotational-slip correction in Odometry predict and RT wheel-arc target

**Completes issue:** `d02-apply-rotational-slip.md`
**Chain:** D2 (depends on 024-004 and 024-005 — with OTOS heading fused and gate recovery
in place, the EKF is robust enough that fixing the predict prior does not destabilize it;
depends on 024-004 for the `setPose` P-prior fix that prevents slip-corrected predictions
from strangling the gate at initialization)

**Note on TURN accuracy:** `TURN 9000` endpoint accuracy is validated in ticket 024-004
(D1), not here. TURN stops on the fused `poseHrad` driven by OTOS heading fusion. D2's
isolated effect shows up in `RT` accuracy and dead-reckoning quality between OTOS
corrections. The hardware AC below uses `RT 9000`, not `TURN 9000`.

## Description

`rotationalSlip` (default 0.74) is defined in `Config.h` and registered in
`ConfigRegistry`/`DefaultConfig` but referenced in zero firmware logic. `Odometry::predict()`
uses raw `(dR - dL) / trackwidthMm` without slip correction. A TURN's HEADING stop fires
when *encoders* say the delta is reached, but the chassis physically rotates ~74% of that
— commanded 90° → ~67° physical. The same error corrupts `poseHrad` for subsequent G
world-frame transforms. `beginRotation()` (RT) likewise computes its encoder-arc target
with no slip term.

`turnScale` and `distScale` are similarly registered but dead; this ticket resolves them
(remove or wire — see open question 4 in architecture-update.md).

The `MockMotor` turn-slip sign is also incorrect for the sim field-profile: it does not
reproduce encoder over-report (scrub), the real failure direction. This ticket corrects it
so the field-profile sim is a valid regression proxy.

## Files to Touch

- `source/control/Odometry.cpp` — `predict()`: replace raw dθ with:
  ```
  float slip = (cfg.rotationalSlip <= 0.0f) ? 1.0f : clamp(cfg.rotationalSlip, 0.5f, 1.0f);
  dTheta = ((dR - dL) / trackwidthMm) * slip;
  ```
  This is the migration-safe form: 0/unset → 1.0, preserving existing exact-profile tests.
  `beginRotation()` (RT) encoder-arc target: divide the arc by the same `slip` value so the
  wheels travel far enough to achieve the commanded angle.
- `source/types/Config.h` — resolve `turnScale` / `distScale`: remove dead fields or wire
  them. Confirm the decision with team-lead (open question 4). If removed: delete from
  `Config.h`, `ConfigRegistry.cpp`, and `tovez.json`; regenerate `DefaultConfig.cpp`.
- `host_tests/MockMotor` (or equivalent) — correct turn-slip sign: mock encoder velocity
  should exceed body rotation (over-report / scrub), not under-report. Set
  `slipTurnExtra ≈ 0.26` in the field-profile fixture.
- `tests/dev/test_ekf.py` — **update in lockstep with Odometry changes**: the Python EKF
  mirror's `predict()` equivalent must apply the same slip correction. Update the field-
  profile test fixture's slip model to use the corrected over-report sign. Any
  `TestSquareFigureEight` or similar test in field-profile mode must pass after this change.

## Acceptance Criteria

- [ ] `Odometry::predict()` applies `cfg.rotationalSlip` with the migration-safe clamp:
  `val <= 0 → 1.0`, otherwise `clamp(val, 0.5, 1.0)`. Existing tests with
  `rotationalSlip = 0` (or absent) continue to pass (treated as 1.0, not 0.5×).
- [ ] `beginRotation()` (RT) divides its encoder-arc target by the same clamped slip value.
- [ ] `turnScale` / `distScale` resolved: either removed from all three locations
  (`Config.h`, `ConfigRegistry.cpp`, `tovez.json` + regenerated `DefaultConfig.cpp`) or
  wired with documented purpose. Team-lead decision recorded in ticket before execution.
- [ ] `MockMotor` turn-slip sign corrected: field-profile sim encodes encoder over-report
  (scrub), not under-report. Field-profile fixture uses `slipTurnExtra ≈ 0.26`.
- [ ] **`tests/dev/test_ekf.py` updated in lockstep:** Python EKF mirror's predict applies
  the same slip correction. Field-profile test fixture uses corrected slip sign.
  `TestSquareFigureEight` in field-profile mode passes with OTOS heading fusion on.
- [ ] **Hardware (isolates D2 — encoder-arc stop, no OTOS fusion turned off):**
  `RT 9000` lands 90° ± 3° physical (measured by protractor or OTOS readout), where
  today it lands ~67° due to uncorrected slip.
- [ ] **Sim (field profile, slip on):** with mock slip on, predicted heading tracks
  mock-body truth after the slip correction; dead-reckoning quality between OTOS
  corrections is visibly improved.
- [ ] Existing exact-profile `host_tests/` pass unmodified.

## Implementation Plan

### Approach

The `predict()` change is a single-line replacement. The `beginRotation()` change is a
single division using the same clamped slip value — extract a helper `effectiveSlip()`
that both call. The MockMotor sign fix requires identifying where the turn slip is applied
and negating / replacing the term. The `turnScale`/`distScale` decision must be confirmed
before execution; if removing, grep for all usages and clean up cleanly.

For the Python mirror: the `predict()` change must be applied to the corresponding Python
function in the same commit, with a test confirming the slip behavior.

### Testing Plan

1. `test_predict_rotational_slip` in `tests/dev/test_ekf.py`: run two predict steps with
   `rotationalSlip=0.74`; assert `_x[2]` (theta) advances by 74% of the encoder dθ.
2. `test_predict_slip_zero_is_identity` in `tests/dev/test_ekf.py`: set
   `rotationalSlip=0.0`; assert slip factor is 1.0 (not 0.0 or 0.5×).
3. Host_tests `test_rt_slip_compensation`: `beginRotation(9000 centi-deg)` with
   `rotationalSlip=0.74`; assert the encoder-arc target is ~9000 / 0.74 ≈ 12162 centi-deg.
4. Field-profile sim square run with D1+D2 both active; assert cumulative heading drift
   after 4 turns < 10°.
5. `uv run pytest tests/dev/test_ekf.py host_tests/`.

### Documentation Updates

If `turnScale`/`distScale` are removed: update any schema docs. Note in `tovez.json`
comments that `rotationalSlip` is now active (was previously dead).
