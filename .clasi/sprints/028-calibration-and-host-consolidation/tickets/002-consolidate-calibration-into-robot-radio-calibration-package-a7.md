---
id: '002'
title: Consolidate calibration into robot_radio/calibration/ package (a7)
status: open
use-cases:
  - SUC-001
depends-on: []
github-issue: ''
issue: a7-consolidate-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 028-002: Consolidate calibration into robot_radio/calibration/ package (a7)

## Description

Calibration logic is duplicated across four files with three independent copies
of `_deep_merge`, three copies of `scale_to_int8` / `_scale_to_int8`, two
copies of `mean_stdev`, and two copies of `_save_config`:

- `host/calibrate_angular.py` (718 lines) — standalone Angular entry point
- `host/calibrate_linear.py` (555 lines) — standalone Linear entry point
- `host/calibrate_verify.py` — verify helper
- `host/robot_radio/io/calibrate.py` (1101 lines) — `rogo calibrate` subcommands

Create `robot_radio/calibration/` with shared helpers and core logic. Reduce
all four files to thin entry points or wiring modules that delegate to the
package. The a8 lint (sprint 025 ticket 003) enforces going forward that no
calibration-output key is registered but unread; this ticket cleans up
existing duplicates and verifies the consolidation satisfies the lint.

## Acceptance Criteria

- [ ] New package `host/robot_radio/calibration/__init__.py` exists.
- [ ] `host/robot_radio/calibration/helpers.py` contains exactly one
      implementation each of: `scale_to_int8`, `int8_to_scale`, `mean_stdev`,
      `deep_merge`, `save_config`, `resolve_save_path`.
- [ ] `host/robot_radio/calibration/push.py` contains `push_calibration(conn_or_proto, config)`
      returning a result dict. Resolves the interface duality (architecture-update.md
      Open Question 3): when passed a `NezhaProtocol`, calls
      `proto.push_calibration(config)`; when passed a bare `SerialConnection`,
      constructs SET commands directly. Both paths tested.
- [ ] `host/robot_radio/calibration/angular.py` contains `calibrate_turns(conn, config, ...)`
      — the interactive turns calibration logic, no duplication with `linear.py`.
- [ ] `host/robot_radio/calibration/linear.py` contains `calibrate_distance(conn, config, ...)`
      — the interactive distance calibration logic.
- [ ] `host/calibrate_angular.py` reduced to a thin entry point (< 30 lines):
      imports `calibrate_turns`, handles CLI args, calls it.
- [ ] `host/calibrate_linear.py` reduced to a thin entry point (< 30 lines).
- [ ] `host/robot_radio/io/calibrate.py` reduced: the `rogo calibrate` subcommand
      wiring remains; calibration math delegates to the package.
- [ ] `cli.py` `_scale_to_int8` removed; replaced with import from
      `robot_radio.calibration.helpers`.
- [ ] Zero duplicated implementations of `scale_to_int8`, `mean_stdev`,
      `deep_merge` anywhere in the codebase (verify with grep).
- [ ] Manual verification: run `python scripts/check_config_sync.py` (if sprint
      025 ticket 003 has landed) and confirm exit 0; or manually confirm all
      calibration-output keys (`OL`, `OA`, `ML`, `MR`, `TW`) are read in
      `source/` firmware.
- [ ] All existing tests pass:
      `uv run --with pytest python -m pytest host_tests/ tests/dev/ -v`

## Implementation Plan

### Approach

1. Create `host/robot_radio/calibration/__init__.py` (empty or re-exports).
2. Write `helpers.py` — move the canonical implementations from
   `host/robot_radio/io/calibrate.py` (it has the most complete versions of
   `_deep_merge`, `_save_config`, `_resolve_save_path`) and the
   `scale_to_int8` / `int8_to_scale` / `mean_stdev` from `calibrate_linear.py`
   (cleanest versions).
3. Write `push.py` — consolidate the push-to-firmware logic. See
   `cli.py::_push_calibration` (lines ~332–410) and
   `robot_mcp.py::_connect()` which calls `_robot._proto.push_calibration`.
   Define `push_calibration(conn_or_proto, config)` with both paths.
4. Write `angular.py` — extract the core interactive logic from
   `host/calibrate_angular.py` into a `calibrate_turns()` function.
5. Write `linear.py` — extract from `host/calibrate_linear.py` into
   `calibrate_distance()`.
6. Reduce `host/calibrate_angular.py` to thin entry point.
7. Reduce `host/calibrate_linear.py` to thin entry point.
8. Reduce `host/robot_radio/io/calibrate.py` — keep subcommand wiring,
   delegate math.
9. Remove `_scale_to_int8` from `cli.py`; add import from helpers.

### Files to create

- `host/robot_radio/calibration/__init__.py`
- `host/robot_radio/calibration/helpers.py`
- `host/robot_radio/calibration/push.py`
- `host/robot_radio/calibration/angular.py`
- `host/robot_radio/calibration/linear.py`

### Files to modify

- `host/calibrate_angular.py` — reduce to thin entry point
- `host/calibrate_linear.py` — reduce to thin entry point
- `host/robot_radio/io/calibrate.py` — remove duplicated helpers, delegate
- `host/robot_radio/io/cli.py` — remove `_scale_to_int8`; add import

### Testing plan

```
uv run --with pytest python -m pytest host_tests/ tests/dev/ -v
```

New unit tests in `tests/dev/test_calibration_helpers.py`:
- `scale_to_int8(1.027)` == 27
- `scale_to_int8(1.0)` == 0
- `int8_to_scale(27)` == 1.027
- `mean_stdev([1.0, 2.0, 3.0])` returns (2.0, 1.0)
- `deep_merge({'a': {'b': 1}}, {'a': {'c': 2}})` merges without overwriting `b`

Integration smoke: `rogo --help` exits 0 (exercises cli.py import path).

### Documentation updates

Add a docstring to `robot_radio/calibration/__init__.py` explaining the
package structure and what each submodule owns.

## Notes

- **Revalidation flag**: this ticket assumes sprint 025 ticket 003 (a8 lint)
  is in place for CI enforcement. If 025 has not yet executed, the acceptance
  criterion uses a manual grep verification instead. Note this when 028 begins.
- Do not wire `push_calibration` into `cli.py` or `robot_mcp.py` in this
  ticket — that is ticket 028-003. This ticket only creates the package and
  ensures the helpers are correct and tested.
