---
id: '001'
title: Package Scaffold
status: in-progress
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: mbdeploy-a-standalone-micro-bit-deploy-package-build-deploy-list-probe.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Package Scaffold

## Description

Create the `mbdeploy/` package directory structure with a valid `pyproject.toml`,
src layout, empty module stubs, and a `README.md`. The goal is a package that
installs cleanly via `pipx install --editable ./mbdeploy` and puts `mbdeploy --help`
on the developer's PATH. No real logic yet — just the scaffold and entry point wiring.

## Acceptance Criteria

- [ ] `mbdeploy/pyproject.toml` exists with `name = "mbdeploy"`, hatchling build
  backend, `requires-python = ">=3.10"`, `dependencies = ["pyocd>=0.44.1",
  "pyserial>=3.5"]`, and `[project.scripts] mbdeploy = "mbdeploy.cli:main"`.
- [ ] `mbdeploy/src/mbdeploy/__init__.py` exists (may be empty or contain version).
- [ ] `mbdeploy/src/mbdeploy/cli.py` exists with a `main()` function that sets up
  argparse with four subcommands (`build`, `deploy`, `list`, `probe`) and exits
  cleanly (subcommands may be stubs that print "not implemented" for now).
- [ ] `mbdeploy/src/mbdeploy/devices.py` exists as an empty stub module.
- [ ] `mbdeploy/src/mbdeploy/builder.py` exists as an empty stub module.
- [ ] `mbdeploy/README.md` exists with basic description and install instructions.
- [ ] `pipx install --editable ./mbdeploy` exits 0.
- [ ] `mbdeploy --help` exits 0 and lists all four subcommands in its output.
- [ ] `mbdeploy build --help`, `mbdeploy deploy --help`, `mbdeploy list --help`,
  `mbdeploy probe --help` all exit 0 without error.

## Implementation Plan

### Approach

Create the directory tree, write `pyproject.toml`, write stub modules, wire the
entry point. Verify with pipx install and a help invocation.

### Files to Create

```
mbdeploy/
├── pyproject.toml
├── README.md
└── src/
    └── mbdeploy/
        ├── __init__.py
        ├── cli.py
        ├── devices.py
        └── builder.py
```

**`mbdeploy/pyproject.toml`:**
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

**`mbdeploy/src/mbdeploy/cli.py`** — argparse scaffold with four subcommands.
Each subcommand parser should define its expected arguments (even if the handler
is a stub) so that `--help` output is correct for future tickets. Define all flags
documented in the issue:
- `build`: `--clean`, `--verbose`, `-j N`, `--build-cmd CMD`
- `deploy`: `[target]`, `--build`, `--clean`, `-j N`, `--force-relay`,
  `--hex PATH`, `--target-mcu nrf52833`, `--config PATH`
- `list`: `--config PATH`
- `probe`: `--config PATH`

**`mbdeploy/README.md`:** Brief description, `pipx install --editable ./mbdeploy`,
list of four subcommands with one-line descriptions.

### Files to Modify

None — this ticket creates new files only.

### Testing Plan

- Run `pipx install --editable ./mbdeploy` and confirm exit 0.
- Run `mbdeploy --help` and confirm output contains `build`, `deploy`, `list`, `probe`.
- Run each subcommand `--help` and confirm it exits 0.
- No unit tests needed for this ticket; the package itself is the test artifact.

### Documentation Updates

`mbdeploy/README.md` is created as part of this ticket.
