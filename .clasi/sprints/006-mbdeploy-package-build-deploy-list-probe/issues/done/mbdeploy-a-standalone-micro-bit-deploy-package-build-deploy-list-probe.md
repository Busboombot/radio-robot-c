---
status: done
sprint: '006'
tickets:
- '001'
- '002'
- '003'
- '004'
---

# `mbdeploy`: a standalone micro:bit deploy package (build / deploy / list / probe)

## Context

The deploy tooling is currently three loose scripts (`scripts/build.py` wrapper,
`scripts/deploy.py`, `scripts/build_and_deploy.py`) plus an ad-hoc registry
(`scripts/lib/known_devices.json`). We're consolidating all of it into a
**separate, pipx-installable Python package named `mbdeploy`**, developed as a
sub-package of this repo so we can refine it against real hardware here, then
move it out and publish it later.

`mbdeploy` identifies every connected micro:bit by its pyOCD **Unique ID**, joins
that to the board's `/dev/cu.*` serial port (via `ioreg`) and its firmware
`DEVICE:` announcement, and keeps a **persistent registry at `config/devices.json`**
(relative to the project you run it in). A `probe` command records each board and
assigns it a **stable enumeration number (1..N)** that never changes for a known
UID. `deploy` then targets a *specific known device* — by enum number, 5-char
micro:bit name (e.g. `gutov`), serial path, or UID — and **refuses to flash a
board recorded as the radio relay** (override available). Selecting a known,
named device is the relay guard.

Decisions (from the user):
- **Separate package `mbdeploy`**, sub-dir of this repo, **pipx-installed**
  (editable) to refine before extraction/publishing.
- **Console command `mbdeploy`** with subcommands `build`/`deploy`/`list`/`probe`.
- **`build` is project-aware but portable**: it runs the project's `./build.py`
  in the current working directory (configurable), since list/probe/deploy are
  generic but CODAL building is project-specific.
- **Remove and consolidate** the old `scripts/*` deploy code into the package.
- Root `build.py` stays put (Dockerfile/upstream use it); `mbdeploy build`
  shells out to it. Registry lives at the project's `config/devices.json`.

## Package layout (src layout, its own distribution)

```
mbdeploy/
├── pyproject.toml            # name=mbdeploy, [project.scripts] mbdeploy=…, deps: pyocd, pyserial
├── README.md
└── src/mbdeploy/
    ├── __init__.py
    ├── cli.py                # argparse: build/deploy/list/probe subcommands + main()
    ├── devices.py            # pyocd list + ioreg join + HELLO probe + registry + resolve
    └── builder.py            # run the project's build command in CWD
```

`mbdeploy/pyproject.toml` (hatchling backend):
```toml
[project]
name = "mbdeploy"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = ["pyocd>=0.44.1", "pyserial>=3.5"]
[project.scripts]
mbdeploy = "mbdeploy.cli:main"
[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
```

Install / refine loop: `pipx install --editable ./mbdeploy` → `mbdeploy` on PATH;
edits reflected live. Run it from the project root so it finds `./build.py`,
`./MICROBIT.hex`, and `./config/devices.json`. All project paths are
**CWD-relative** (with `--config`/`--hex`/`--build-cmd` overrides) so the package
stays portable once extracted.

## `devices.py` (ported/cleaned from the current `scripts/lib/device_link.py`)
Reuse the proven primitives, repointed at `config/devices.json`:
- `flashable_probes()` → `[{uid, description}]` (pyOCD API, CLI fallback).
- `port_serial_map(known)` → `{uid: /dev/cu.*}` via `ioreg` (macOS; `{}` elsewhere).
- `probe_type(port)` → `{role, common_name, device_name, serial, raw}` (HELLO).
- `is_relay(role)` — generic: matches `RELAY`/`BRIDGE` (catches `RADIOBRIDGE`).
  (Drop the project-specific `is_robot`/`Nezha2` rule — auto-pick = the unique
  non-relay device instead.)
- New registry layer at `config/devices.json`:
  `load_devices()`, `save_devices()`, `assign_enum(devices, uid)` (existing enum
  or `max+1`, min 1), `probe_all(config_path)` (join + announce, merge fields +
  enum, preserve prior announcement on a busy/silent port, never delete entries,
  save, return list), `resolve_target(token, devices)`.

Registry entry schema:
```json
{
  "9906...052820": {
    "enum": 1, "uid": "9906...052820",
    "port": "/dev/cu.usbmodem21421202",
    "announcement": "DEVICE:Nezha2:gutov:microbit:9990001234",
    "role": "Nezha2", "common_name": "gutov",
    "device_name": "microbit", "serial": "9990001234"
  }
}
```
Committed to git. `port` is host-specific and refreshed every probe/deploy;
flashing always re-confirms the live UID→port before acting.

## `cli.py` subcommands
- **`build [--clean] [--verbose] [-j N] [--build-cmd CMD]`** → `builder.run()`:
  invoke the project build (default `python3 build.py` in CWD, mapping flags) →
  `MICROBIT.hex`.
- **`list`** → live `flashable_probes()` + `port_serial_map()`; table of UID +
  port, annotated with enum/role/name from `config/devices.json` when known.
  Read-only.
- **`probe [--config PATH]`** → `devices.probe_all()`; print the enum/name/role/
  port/uid table. Populates `config/devices.json`.
- **`deploy [target] [--build] [--clean] [-j N] [--force-relay] [--hex PATH] [--target-mcu nrf52833] [--config PATH]`**
  → `resolve_target()` (or auto-pick the unique non-relay device when omitted;
  error if ambiguous). **Refuse if `is_relay` unless `--force-relay`.** Confirm
  the UID is in the live probe list (else "device not connected"). If `--build`,
  build first. Flash: `pyocd flash -t <mcu> --uid <uid> <hex>` then
  `pyocd reset -t <mcu> --uid <uid>`.

### Target-resolution precedence
digits→enum · starts `/dev/` or contains `/`→serial path · 40–52 hex→UID ·
else→5-char name (match `common_name` **or** `device_name`, since the codename
sits in different announcement fields for robot vs relay). Resolve against
`config/devices.json`; re-confirm the flash UID against the live probe list.

## Remove (consolidated into the package)
- `scripts/deploy.py`, `scripts/build_and_deploy.py`, `scripts/build.py`,
  `scripts/lib/device_link.py`, `scripts/lib/known_devices.json` (delete if
  present). Remove now-empty `scripts/lib/` and `scripts/` (root `build.py`
  stays at repo root; `tests/` is unaffected).

## Update
- **`justfile`**: add `mbd-install: pipx install --editable ./mbdeploy`; replace
  `scripts-build`/`deploy`/`build-deploy` with `build`, `build-clean`, `list`,
  `probe`, `deploy *args`, `build-deploy *args` → all calling the installed
  `mbdeploy` command.
- **Root `pyproject.toml`**: drop `pyocd` (now owned by `mbdeploy`); keep
  `pyserial` (used by `tests/rogo.py`).
- **`README.md`**: document `pipx install --editable ./mbdeploy` and the four
  subcommands + target selectors.
- **`Dockerfile`**: unchanged (still `python3 build.py`).

## Verification
- `pipx install --editable ./mbdeploy` then `mbdeploy --help` lists the four
  subcommands.
- `mbdeploy list` → connected board's UID + `/dev/cu.*` port (today: UID
  `9906…cfd6c…` ↔ `/dev/cu.usbmodem21421202`).
- `mbdeploy probe` → writes `config/devices.json` with `enum: 1` for the board;
  re-running keeps enum=1 and refreshes port/announcement. (Bench rig is
  currently silent to HELLO → `role` may be blank; enum/uid/port still populate.)
- `mbdeploy build` → produces `MICROBIT.hex` via the root `build.py`.
- `mbdeploy deploy 1` / `deploy gutov` / `deploy /dev/cu.usbmodem…` resolve to the
  same UID; `deploy --build 1` builds then flashes; a board recorded as relay is
  refused without `--force-relay`; an enum not in the live probe list errors
  "device not connected".
- Unit-level: monkeypatch `flashable_probes`/registry to assert relay refusal,
  unique-device auto-pick, and enum/name/path/uid resolution (mirrors the
  simulated checks used while building the current deploy.py).
- `just probe`, `just list`, `just deploy 1`, `just build-deploy -- 1` smoke-test
  the recipes against the installed command.
