---
id: '010'
title: Calibration scripts rewritten to v2 + relay
status: done
use-cases:
- SUC-001
- SUC-003
- SUC-007
- SUC-008
depends-on:
- '003'
- '004'
- '005'
- 008
- 009
github-issue: ''
issue: ''
completes_issue: false
---

# Calibration scripts rewritten to v2 + relay

## Description

Port/rewrite the calibration scripts from the prior system to work over the
v2 relay (not direct serial), parsing v2 TLM `pose=` (now in mm, after T03)
instead of the dead `SO` stream.

Prior scripts live at:
`/Volumes/Proj/proj/league-projects/scratch/radio-robot/test/calibrate/`
- `calibrate_linear.py` — drives measured distances, computes `otos_linear_scale`
  and verifies `mmPerDeg` using OTOS as ground truth.
- `calibrate_angular.py` — spins measured angles, computes `otos_angular_scale`
  and per-direction turn gains using OTOS/camera as ground truth.

Both must be ported to v2 protocol and adapted to use the relay data plane.

## Files to Create/Modify

- **`host/calibrate_linear.py`** (new) — port from prior system:
  - Connect via relay (`SerialConnection` in relay mode, robot enum 2).
  - Drive a measured distance using `D 200 200 <mm>`.
  - Wait for `EVT done D`, then issue `SNAP` and parse `TLM ... pose=x,y,h`.
  - Compare `pose=` x (mm) to commanded distance.
  - Compute `otos_linear_scale = commanded_mm / pose_x * current_scale`.
  - Optionally: verify `mmPerDeg` by comparing encoder-derived distance to tape measure.
  - Write updated scale back to `data/robots/<robot>.json`.

- **`host/calibrate_angular.py`** (new) — port from prior system:
  - Connect via relay.
  - Spin CCW and CW by known angles (e.g. 360°).
  - Parse `TLM ... pose=x,y,h` (h in centidegrees) for final heading.
  - Compare h to commanded angle.
  - Compute `otos_angular_scale = commanded_deg / actual_deg * current_scale`.
  - Compute per-direction gain: `rotationGainNeg = commanded_cw / actual_cw`.
  - Write computed values to `data/robots/<robot>.json`.

- **`host/robot_radio/sensors/odom_tracker.py`** — add `parse_tlm(line)` helper:
  ```python
  def parse_tlm(line: str) -> dict | None:
      """Parse 'TLM t=... pose=x,y,h ...' -> {'pose': (x, y, h), ...} or None."""
  ```
  The existing `parse_so()` function is left in place (not removed).

- **`host/robot_radio/sensors/calibration.py`** — update docstring to reference
  v2 TLM instead of SO stream.

- **`host/robot_radio/io/calibrate.py`** — update any `parse_so` calls to use
  `parse_tlm` in new code paths.

## Approach

1. Read prior `calibrate_linear.py` and `calibrate_angular.py` for algorithm.
2. Read the relay connection API (`SerialConnection`, `send`, `read_lines`).
3. Implement `parse_tlm()` in `odom_tracker.py`.
4. Port the scripts, replacing `SO` stream parsing with `SNAP`/TLM parsing.
5. Dry-run the script structure offline (no robot) with mock connection to confirm
   the flow is correct.
6. Offline test: `parse_tlm` unit test with a sample TLM line.
7. Full end-to-end (bench-deferred to T11): run scripts on relay, confirm
   output values are reasonable.

## Acceptance Criteria

- [x] `host/calibrate_linear.py` exists and is importable (no syntax errors).
- [x] `host/calibrate_angular.py` exists and is importable.
- [x] `parse_tlm()` in `odom_tracker.py` correctly parses a `TLM t=... pose=x,y,h` line.
- [x] `parse_tlm()` unit test passes.
- [x] Scripts do not import `parse_so` or reference the dead `SO` stream.
- [x] `uv run pytest` passes (no regressions).
- [x] `sensors/calibration.py` docstring references v2 TLM.
- [ ] (Bench deferred to T11) Scripts run end-to-end over relay; emit recommended config values; `data/robots/<robot>.json` is updated.

## Testing

- **New tests**: `parse_tlm()` unit test in `tests/test_odom_tracker.py`.
- **Existing tests to run**: full `uv run pytest`
- **Verification command**: `uv run pytest`
