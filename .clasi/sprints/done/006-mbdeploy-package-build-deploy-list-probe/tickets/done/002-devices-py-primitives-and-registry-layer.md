---
id: '002'
title: "devices.py \u2014 Primitives and Registry Layer"
status: done
use-cases:
- SUC-002
- SUC-003
- SUC-005
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# devices.py — Primitives and Registry Layer

## Description

Implement `mbdeploy/src/mbdeploy/devices.py` in full. This ticket ports the
proven device-link primitives from `scripts/lib/device_link.py` into the new
package (with the `is_robot`/`ROBOT_ROLES` coupling removed) and adds the new
persistent registry layer on top. The result is the complete device domain for
the package — all other modules delegate here.

## Acceptance Criteria

### Primitives (ported from `scripts/lib/device_link.py`)

- [x] `flashable_probes() -> list[dict[str, str]]` returns `[{uid, description}]`
  using `ConnectHelper.get_all_connected_probes()` with a `pyocd list` CLI fallback.
- [x] `port_serial_map(known: set[str] | None = None) -> dict[str, str]` returns
  `{uid: "/dev/cu.*"}` via `ioreg -r -c IOUSBHostDevice -l` on macOS; returns `{}`
  on non-macOS or if `ioreg` is unavailable.
- [x] `probe_type(port: str, timeout_s: float = 1.6) -> dict | None` opens the
  serial port at 115200 baud with DTR/RTS False, sends `HELLO\n`, returns
  `{role, common_name, device_name, serial, raw}` from the first `DEVICE:` line,
  or `None` on timeout/busy/no-announcement.
- [x] `is_relay(role: str | None) -> bool` returns True if `role` contains `RELAY`
  or `BRIDGE` (case-insensitive); returns False for None or empty.
- [x] `is_robot` is NOT present (removed — not needed in the generic package).

### Registry Layer

- [x] `load_devices(config_path: Path) -> dict[str, dict]` reads the registry
  JSON; returns `{}` on missing or invalid file.
- [x] `save_devices(devices: dict[str, dict], config_path: Path) -> None` writes
  the registry; creates parent directory if needed.
- [x] `assign_enum(devices: dict[str, dict], uid: str) -> int` returns the existing
  enum for `uid` if present, or `max(existing enums) + 1` (minimum 1) for a new UID.
- [x] `probe_all(config_path: Path) -> list[dict]` joins `flashable_probes()` +
  `port_serial_map()`, probes each port via `probe_type()`, merges announcement fields
  and enum into the registry, preserves prior announcement on a busy/silent port
  (does not clear existing `role`/`common_name` etc.), never deletes entries, saves
  registry, returns the updated list of device dicts.
- [x] `resolve_target(token: str, devices: dict[str, dict]) -> dict` resolves using
  this precedence:
  1. Pure digits → match by `enum` field
  2. Starts with `/dev/` or contains `/` → match by `port` field
  3. 40–52 hex characters → match by `uid` field
  4. Otherwise → match `common_name` OR `device_name` (case-insensitive)
  Raises `ValueError` with a descriptive message if not found.

### Registry Entry Schema

Every entry written by `probe_all` must have these keys (blank/None for unknown):

```json
{
  "<uid>": {
    "enum": 1,
    "uid": "<uid>",
    "port": "/dev/cu.usbmodem...",
    "announcement": "DEVICE:...",
    "role": "...",
    "common_name": "...",
    "device_name": "...",
    "serial": "..."
  }
}
```

- [x] `port` is always refreshed from the current `port_serial_map()` result
  even if HELLO probe fails.
- [x] If `probe_type` returns `None`, existing `announcement`/`role`/`common_name`/
  `device_name`/`serial` fields are preserved unchanged.
- [x] A board with no serial port in the map still gets an entry with `port: null`.

## Implementation Plan

### Approach

Port the four primitives from `scripts/lib/device_link.py` with minimal changes:
- Remove `Device` dataclass and `enumerate_devices`/`find_by_uid` (superseded).
- Remove `is_robot`/`ROBOT_ROLES`.
- Remove old `load_registry`/`save_registry`/`remember` (replaced by new layer).
- Keep `flashable_probes`, `_flashable_probes_cli`, `port_serial_map`, `probe_type`,
  `is_relay` functionally identical to the existing implementations.

Then add the five new registry functions on top.

### Files to Modify

- `mbdeploy/src/mbdeploy/devices.py` — replace stub with full implementation.

### Key Implementation Notes

**`probe_all` merge logic (pseudocode):**
```python
devices = load_devices(config_path)
probes = flashable_probes()
uids = {p["uid"] for p in probes}
ports = port_serial_map(uids)
for p in probes:
    uid = p["uid"]
    entry = devices.get(uid, {})
    entry["uid"] = uid
    entry["port"] = ports.get(uid)           # always refresh
    if "enum" not in entry:
        entry["enum"] = assign_enum(devices, uid)
    port = entry.get("port")
    info = probe_type(port) if port else None
    if info:
        entry["announcement"] = info["raw"]
        entry["role"]         = info["role"]
        entry["common_name"]  = info["common_name"]
        entry["device_name"]  = info["device_name"]
        entry["serial"]       = info["serial"]
    # else: preserve existing announcement fields
    devices[uid] = entry
save_devices(devices, config_path)
return list(devices.values())
```

**`resolve_target` name-matching note:**
The 5-char codename appears in different announcement fields for robot vs relay:
- Robot: `DEVICE:Nezha2:<name>:microbit:...` → stored as `common_name`
- Relay: `DEVICE:RADIOBRIDGE:relay:<name>:...` → stored as `device_name`

Name resolution must check both fields:
```python
for entry in devices.values():
    if (entry.get("common_name", "").lower() == token.lower() or
            entry.get("device_name", "").lower() == token.lower()):
        return entry
raise ValueError(f"No device found matching '{token}'")
```

**`assign_enum` logic:**
```python
def assign_enum(devices: dict, uid: str) -> int:
    if uid in devices and "enum" in devices[uid]:
        return devices[uid]["enum"]
    existing = [e["enum"] for e in devices.values() if "enum" in e]
    return max(existing, default=0) + 1
```

### Testing Plan

Unit tests are written in ticket 003. The programmer should import-check:
- `from mbdeploy.devices import (flashable_probes, port_serial_map, probe_type,
  is_relay, load_devices, save_devices, assign_enum, probe_all, resolve_target)`
  imports cleanly with no errors.

Quick smoke checks (no hardware needed):
- `is_relay("RADIOBRIDGE")` returns True
- `is_relay("Nezha2")` returns False
- `assign_enum({}, "abc")` returns 1
- `assign_enum({"abc": {"enum": 1}}, "abc")` returns 1
- `assign_enum({"abc": {"enum": 1}}, "xyz")` returns 2
- `resolve_target("1", {"uid1": {"enum": 1, "uid": "uid1"}})` returns the entry
- `resolve_target("gutov", {"uid1": {"enum": 1, "common_name": "gutov"}})` returns the entry

### Documentation Updates

Module-level docstring should describe the registry invariants: entries are never
deleted, `port` is always refreshed, prior announcement is preserved on silent port,
`enum` is assigned once and never changes.
