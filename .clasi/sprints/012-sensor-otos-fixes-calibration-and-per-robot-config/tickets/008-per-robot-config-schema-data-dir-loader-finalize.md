---
id: 008
title: Per-robot config schema + data dir + loader finalize
status: done
use-cases:
- SUC-008
depends-on: []
github-issue: ''
issue: ''
completes_issue: false
---

# Per-robot config schema + data dir + loader finalize

## Description

Host-only ticket (no firmware changes). The loader (`robot_config.py`) exists
with the `RobotConfig` Pydantic model, but:
- No `data/robots/<robot>.json` file exists for the active nezha robot.
- No `robot_config.schema.json` exists in this repo (must port from prior system).
- Robot matching by v2 `ID` response is not wired up (`connection.device_announcement_name`
  field exists but is not used for matching during connect).
- `rotation_gain_neg`/`rotation_offset_deg_neg` fields may need a schema_version bump.

## Files to Create/Modify

- **`data/robots/robot_config.schema.json`** — port from prior system at
  `/Volumes/Proj/proj/league-projects/scratch/radio-robot/data/robots/robot_config.schema.json`.
  Update to match current `robot_config.py` Pydantic model fields.

- **`data/robots/<robot-name>.json`** — create the active robot's config file
  (the robot name is the micro:bit friendly name; confirm on hardware).
  Seed with known-good values:
  - `schema_version: 2`
  - `identity.robot_name`: the micro:bit friendly name
  - `geometry.trackwidth`: 126
  - `calibration.otos_linear_scale`: 1.05
  - `calibration.otos_angular_scale`: 0.987
  - `calibration.mm_per_wheel_deg_left`: 0.487
  - `calibration.mm_per_wheel_deg_right`: 0.481
  - `calibration.rotation_gain_neg`: 1.17
  - `calibration.rotational_slip`: 0.74
  - `connection.device_announcement_name`: the micro:bit friendly name (from HELLO/ID)

- **`data/robots/active_robot.json`** — create pointer file:
  `{"path": "data/robots/<robot-name>.json"}`

- **`host/robot_radio/config/robot_config.py`** — finalize:
  - Add `schema_version: 2` handling (v1 files load without error, just no new fields).
  - Add `match_robot_by_id(id_response: str) -> Optional[RobotConfig]` function
    that parses the v2 `ID` response (e.g. `ID model=Nezha2 name=TOVEZ serial=...`)
    and returns the config whose `connection.device_announcement_name` matches `name=`.
    Falls back to `get_robot_config()` if no match (backward compat).

## Approach

1. Read the prior system schema and nezha-1.json for reference.
2. Create the robot JSON file with known-good values (get the micro:bit friendly
   name from the robot or from the session cache).
3. Port and update the JSON schema to match `robot_config.py`.
4. Add `match_robot_by_id()` to `robot_config.py`.
5. Write a unit test: parse a sample ID response string, confirm the correct
   config is returned.
6. Verify `get_robot_config()` still works for the active robot via the
   `active_robot.json` pointer.

## Acceptance Criteria

- [x] `data/robots/<robot>.json` exists with known-good calibration values.
- [x] `data/robots/active_robot.json` pointer file resolves to the robot JSON.
- [x] `data/robots/robot_config.schema.json` exists and validates the robot JSON.
- [x] `get_robot_config()` returns the active robot config.
- [x] `match_robot_by_id("ID model=Nezha2 name=<name> serial=...")` returns the matching config.
- [x] Unit test for `match_robot_by_id()` passes.
- [x] `uv run pytest` passes.

## Testing

- **New tests**: `tests/test_robot_config.py` — test load, match_by_id, schema validation.
- **Verification command**: `uv run pytest tests/test_robot_config.py`
