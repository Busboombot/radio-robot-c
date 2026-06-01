---
status: final
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 006 Use Cases

## SUC-001: Install mbdeploy and Verify Entry Point

- **Actor**: Developer
- **Preconditions**: Python 3.10+, pipx, and pyOCD available; `mbdeploy/` directory
  exists in the project root with a valid `pyproject.toml`.
- **Main Flow**:
  1. Developer runs `pipx install --editable ./mbdeploy` from the project root.
  2. pipx creates an isolated environment and installs the package with its
     dependencies (pyocd, pyserial).
  3. Developer runs `mbdeploy --help`.
  4. Help output lists four subcommands: `build`, `deploy`, `list`, `probe`.
- **Postconditions**: `mbdeploy` is on the developer's PATH and ready to use.
- **Acceptance Criteria**:
  - [ ] `pipx install --editable ./mbdeploy` exits 0.
  - [ ] `mbdeploy --help` exits 0 and mentions all four subcommands.
  - [ ] Edits to `mbdeploy/src/mbdeploy/` are reflected without reinstalling.

---

## SUC-002: List Connected Boards

- **Actor**: Developer
- **Preconditions**: One or more micro:bit boards connected via USB; `mbdeploy` installed.
- **Main Flow**:
  1. Developer runs `mbdeploy list` from the project root.
  2. Command calls `flashable_probes()` to get all pyOCD UIDs and their descriptions.
  3. Command calls `port_serial_map()` to get the UID-to-port mapping via `ioreg`.
  4. Command prints a table: UID, port, and (if known from registry) enum, role, and name.
- **Postconditions**: Developer sees which boards are connected and their identities.
- **Acceptance Criteria**:
  - [ ] `mbdeploy list` shows at least one row with a valid UID and `/dev/cu.*` port.
  - [ ] Boards with entries in `config/devices.json` show their enum number and name.
  - [ ] Command is read-only; `config/devices.json` is not modified.
  - [ ] Command exits 0 even if no boards are connected (prints empty table).

---

## SUC-003: Probe Boards and Populate Registry

- **Actor**: Developer
- **Preconditions**: One or more micro:bit boards connected via USB; `mbdeploy` installed.
- **Main Flow**:
  1. Developer runs `mbdeploy probe` from the project root.
  2. Command calls `probe_all()`: enumerates probes, maps to ports, opens each serial
     port, sends `HELLO\n`, and reads the `DEVICE:` announcement.
  3. New boards get a new `enum` number (`max+1`, minimum 1); existing boards keep theirs.
  4. Announcement fields (role, common_name, device_name, serial) are merged into the
     registry entry; a port that is silent or busy preserves the prior announcement.
  5. Registry is saved to `config/devices.json`.
  6. Command prints the updated table: enum, name, role, port, UID.
- **Postconditions**: `config/devices.json` exists and contains an entry for every
  currently connected board; existing enum numbers are unchanged.
- **Acceptance Criteria**:
  - [ ] After first `mbdeploy probe`, `config/devices.json` exists with `"enum": 1` for
    the board.
  - [ ] Re-running `mbdeploy probe` preserves enum=1 and refreshes port/announcement.
  - [ ] A board that is silent to HELLO still gets an entry with enum/uid/port populated.
  - [ ] Entries for previously seen boards are never deleted.

---

## SUC-004: Build Firmware

- **Actor**: Developer
- **Preconditions**: Docker available, project root contains `build.py`, `mbdeploy` installed.
- **Main Flow**:
  1. Developer runs `mbdeploy build` (optionally `--clean`, `--verbose`, `-j N`,
     `--build-cmd CMD`).
  2. Command shells out to `python3 build.py` (or the `--build-cmd` value) in the
     current working directory, passing appropriate flags.
  3. The project build runs and produces `MICROBIT.hex` at the project root.
- **Postconditions**: `MICROBIT.hex` is present and up to date.
- **Acceptance Criteria**:
  - [ ] `mbdeploy build` exits with the same code as the underlying build command.
  - [ ] `mbdeploy build --clean` passes the clean flag to the build.
  - [ ] `mbdeploy build --build-cmd ./my_build.sh` runs the alternate command.
  - [ ] Root `build.py` is not deleted or modified by this sprint.

---

## SUC-005: Deploy Firmware to a Specific Board

- **Actor**: Developer
- **Preconditions**: Board is connected, probed (enum assigned), `MICROBIT.hex` exists,
  `mbdeploy` installed.
- **Main Flow**:
  1. Developer runs `mbdeploy deploy <target>` where `<target>` is one of:
     - A digit string (enum number, e.g. `1`)
     - A `/dev/` path or path containing `/`
     - A 40-52 hex character UID
     - A 5-char micro:bit name (e.g. `gutov`)
  2. Command resolves the target to a UID via `resolve_target()` using
     `config/devices.json`.
  3. Command checks if the resolved device is marked as relay; if so, refuses unless
     `--force-relay` is given.
  4. Command confirms the resolved UID is present in the live `flashable_probes()` list.
  5. Command runs `pyocd flash -t <mcu> --uid <uid> <hex>` then
     `pyocd reset -t <mcu> --uid <uid>`.
- **Postconditions**: The targeted board has the new firmware flashed; relay was protected.
- **Acceptance Criteria**:
  - [ ] `mbdeploy deploy 1`, `deploy gutov`, `deploy /dev/cu.usbmodem...` all resolve to
    the same UID and succeed.
  - [ ] `mbdeploy deploy --build 1` builds first, then flashes.
  - [ ] A board with `role` containing `RELAY` or `BRIDGE` is refused; exit non-zero.
  - [ ] `mbdeploy deploy --force-relay <relay-target>` succeeds despite relay role.
  - [ ] If resolved UID is not in live probe list, exits with "device not connected".
  - [ ] Name resolution checks both `common_name` and `device_name` fields.

---

## SUC-006: Cutover from Legacy Scripts

- **Actor**: Developer
- **Preconditions**: `mbdeploy` installed; old scripts deleted; `justfile` and
  `pyproject.toml` updated.
- **Main Flow**:
  1. Developer runs `just mbd-install` to install the package.
  2. Developer uses `just build`, `just list`, `just probe`, `just deploy 1`,
     `just build-deploy -- 1` as drop-in replacements for the old script-based recipes.
  3. Root `pyproject.toml` no longer lists `pyocd` as a dependency.
  4. Old `scripts/` directory is absent.
- **Postconditions**: All deploy workflows go through `mbdeploy`; no old scripts remain.
- **Acceptance Criteria**:
  - [ ] `scripts/deploy.py`, `scripts/build_and_deploy.py`, `scripts/build.py`,
    `scripts/lib/` are all deleted.
  - [ ] `scripts/` directory itself is removed (if empty).
  - [ ] `justfile` has a `mbd-install` recipe.
  - [ ] `just build`, `just list`, `just probe`, `just deploy *args`,
    `just build-deploy *args` all call `mbdeploy`.
  - [ ] Root `pyproject.toml` does not list `pyocd`; `pyserial` remains.
  - [ ] `README.md` documents `pipx install --editable ./mbdeploy` and the four
    subcommands.
  - [ ] `Dockerfile` is unchanged.
