---
id: '003'
title: cli.py, builder.py, and Unit Tests
status: done
use-cases:
- SUC-002
- SUC-003
- SUC-004
- SUC-005
depends-on:
- '002'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# cli.py, builder.py, and Unit Tests

## Description

Implement the full `cli.py` entry point, `builder.py` build shim, and all unit
tests. This ticket makes `mbdeploy` fully functional: the four subcommands all
have real implementations, and the relay-guard / target-resolution / auto-pick
logic is tested without requiring connected hardware.

## Acceptance Criteria

### `builder.py`

- [x] `builder.run(clean=False, verbose=False, jobs=None, build_cmd=None)` shells
  out to `python3 build.py` in CWD by default (or `build_cmd` if given), mapping
  `--clean`, `--verbose`, and `-j N` flags to the corresponding `build.py` CLI args.
- [x] Returns the subprocess exit code (not raising on failure).
- [x] Emits an informative error message if `build.py` is not found in CWD and no
  `--build-cmd` was given.

### `cli.py` — `build` subcommand

- [x] Calls `builder.run()` with the parsed flags.
- [x] Exits with the same code as the build subprocess.

### `cli.py` — `list` subcommand

- [x] Calls `flashable_probes()` and `port_serial_map()` from `devices.py`.
- [x] Loads `config/devices.json` (via `load_devices`) for annotation.
- [x] Prints a table: enum (if known), UID, port, role, name. One row per probe.
- [x] Exits 0 even if no probes are found (prints empty table or "no devices found").
- [x] Does NOT write to `config/devices.json`.

### `cli.py` — `probe` subcommand

- [x] Calls `probe_all(config_path)` from `devices.py`.
- [x] Prints the updated table: enum, UID, port, role, name.
- [x] Exits 0.

### `cli.py` — `deploy` subcommand

- [x] If `[target]` is omitted, auto-picks the unique non-relay device from
  `config/devices.json`; errors if zero or more than one non-relay device.
- [x] Resolves `[target]` via `resolve_target(token, devices)`.
- [x] Refuses if `is_relay(entry["role"])` unless `--force-relay` is given; exits
  non-zero with a clear error message.
- [x] Re-confirms the resolved UID is in the live `flashable_probes()` list; exits
  non-zero with "device not connected: <uid>" if not found.
- [x] If `--build` is given, runs `builder.run()` first; stops if build fails.
- [x] Flashes via `pyocd flash -t <mcu> --uid <uid> <hex>` then
  `pyocd reset -t <mcu> --uid <uid>` using `subprocess.run`.
- [x] `--hex PATH` overrides the hex file path (default `MICROBIT.hex` in CWD).
- [x] `--target-mcu` overrides the MCU target (default `nrf52833`).
- [x] `--config PATH` overrides the registry path.

### Unit Tests

Tests must be written in `mbdeploy/tests/test_devices.py` (or `test_cli.py` if
testing CLI argument parsing). All tests must pass without connected hardware.

- [x] **Relay refusal**: Monkeypatch `flashable_probes` to return one relay UID and
  `load_devices` to return a registry entry with relay role. Assert `deploy` exits
  non-zero when `--force-relay` is absent.
- [x] **Force relay override**: Same setup as above; `deploy --force-relay` proceeds
  past the relay check (may fail later at pyocd step; test only the guard logic).
- [x] **Unique-device auto-pick**: Monkeypatch to return exactly one non-relay device
  and one relay device. Assert auto-pick (no target arg) selects the non-relay device.
- [x] **Ambiguous auto-pick**: Two non-relay devices in registry + probes. Assert
  auto-pick errors with an ambiguous message.
- [x] **Target resolution — enum**: `resolve_target("1", devices)` where one entry has
  `enum=1`. Returns the correct entry.
- [x] **Target resolution — path**: `resolve_target("/dev/cu.usbmodem123", devices)`
  where one entry has `port="/dev/cu.usbmodem123"`. Returns the correct entry.
- [x] **Target resolution — UID**: `resolve_target("9906" + "a"*36, devices)` where
  one entry has that UID. Returns the correct entry. (Use a 40-char hex string.)
- [x] **Target resolution — name via `common_name`**: `resolve_target("gutov", devices)`
  where one entry has `common_name="gutov"`. Returns the correct entry.
- [x] **Target resolution — name via `device_name`**: `resolve_target("relay1", devices)`
  where one entry has `device_name="relay1"` but `common_name="relay"`. Returns entry.
- [x] **Device not connected**: Monkeypatch `flashable_probes` to return empty list
  while registry has a known UID. Assert deploy exits non-zero with "device not
  connected" message.
- [x] **`is_relay` checks**: Unit test `is_relay` directly:
  - `is_relay("RADIOBRIDGE")` → True
  - `is_relay("RADIORELAY")` → True
  - `is_relay("Nezha2")` → False
  - `is_relay(None)` → False
  - `is_relay("")` → False

## Implementation Plan

### Approach

Implement `builder.py` first (simplest). Then fill in `cli.py` subcommand handlers
using the completed `devices.py` API. Write unit tests last, using `pytest` with
`monkeypatch` or `unittest.mock.patch`.

### Files to Modify

- `mbdeploy/src/mbdeploy/builder.py` — replace stub with full implementation.
- `mbdeploy/src/mbdeploy/cli.py` — replace stub subcommand handlers with real logic.

### Files to Create

- `mbdeploy/tests/__init__.py` (empty)
- `mbdeploy/tests/test_devices.py` — unit tests for registry logic, relay guard,
  target resolution, and auto-pick.

### Key Implementation Notes

**`builder.py` invocation pattern:**
```python
import subprocess, sys
from pathlib import Path

def run(clean=False, verbose=False, jobs=None, build_cmd=None):
    if build_cmd:
        cmd = build_cmd.split()
    else:
        if not Path("build.py").exists():
            print("Error: build.py not found in CWD. Use --build-cmd.", file=sys.stderr)
            return 1
        cmd = [sys.executable, "build.py"]
    if clean:   cmd.append("--clean")
    if verbose: cmd.append("--verbose")
    if jobs:    cmd += ["-j", str(jobs)]
    return subprocess.run(cmd).returncode
```

**`deploy` relay guard and live-probe check:**
```python
entry = resolve_target(target_token, devices)   # or auto-picked
if is_relay(entry.get("role")) and not force_relay:
    print(f"Error: {entry.get('common_name') or entry['uid']} is a relay. "
          "Use --force-relay to override.", file=sys.stderr)
    sys.exit(1)
live_uids = {p["uid"] for p in flashable_probes()}
if entry["uid"] not in live_uids:
    print(f"Error: device not connected: {entry['uid']}", file=sys.stderr)
    sys.exit(1)
```

**Auto-pick logic:**
```python
non_relay = [e for e in devices.values() if not is_relay(e.get("role"))]
if len(non_relay) == 0:
    print("Error: no non-relay devices in registry. Run 'mbdeploy probe' first.",
          file=sys.stderr)
    sys.exit(1)
if len(non_relay) > 1:
    names = [e.get("common_name") or e["uid"][:8] for e in non_relay]
    print(f"Error: ambiguous — multiple non-relay devices: {names}. Specify a target.",
          file=sys.stderr)
    sys.exit(1)
entry = non_relay[0]
```

**Test structure (pytest with monkeypatch):**
Tests should patch `mbdeploy.devices.flashable_probes` and `mbdeploy.devices.load_devices`
(or use a tmp_path fixture for `config/devices.json`) to avoid any hardware dependency.
The relay guard and auto-pick logic may also be tested by calling the CLI via
`subprocess.run(["mbdeploy", ...])` with appropriate env/cwd if integration-level
coverage is preferred.

### Testing Plan

- **Verification command**: `cd mbdeploy && python -m pytest tests/ -v`
  (or `uv run pytest mbdeploy/tests/ -v` from repo root)
- All tests must pass without connected hardware.
- Tests do not require a real `config/devices.json` — use `tmp_path` fixtures.

### Documentation Updates

No new documentation needed. The `mbdeploy/README.md` created in ticket 001 already
documents the subcommands; update it if any flags change during implementation.
