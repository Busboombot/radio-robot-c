---
id: '004'
title: "Cutover \u2014 Remove scripts, Update justfile and pyproject"
status: done
use-cases:
- SUC-006
depends-on:
- '003'
github-issue: ''
issue: mbdeploy-a-standalone-micro-bit-deploy-package-build-deploy-list-probe.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Cutover — Remove scripts, Update justfile and pyproject

## Description

With `mbdeploy` fully implemented and tested (tickets 001-003), this ticket
completes the transition: delete the old `scripts/` deploy code, update the
`justfile` to call `mbdeploy`, drop `pyocd` from root `pyproject.toml`, and
update `README.md`. After this ticket, all deploy workflows go through
the installed `mbdeploy` package and the old scripts no longer exist.

## Acceptance Criteria

### Delete Old Scripts

- [ ] `scripts/deploy.py` is deleted.
- [ ] `scripts/build_and_deploy.py` is deleted.
- [ ] `scripts/build.py` is deleted (the thin wrapper; root `build.py` stays).
- [ ] `scripts/lib/device_link.py` is deleted.
- [ ] `scripts/lib/known_devices.json` is deleted if it exists.
- [ ] `scripts/lib/` directory is deleted (rmdir, must be empty after above).
- [ ] `scripts/__pycache__/` and any `.pyc` files under `scripts/` are removed.
- [ ] `scripts/` directory itself is removed if empty.
- [ ] Root `build.py` is NOT deleted or modified.

### `justfile` Updates

Current recipes to remove or replace:
- `scripts-build` (line ~29) — remove entirely (superseded by `build` recipe).
- `deploy *args=''` (line ~32) — replace body with `mbdeploy deploy {{args}}`.
- `build-deploy *args=''` (line ~35) — replace body with
  `mbdeploy build && mbdeploy deploy {{args}}`.

New recipe to add:
```
mbd-install:
    pipx install --editable ./mbdeploy
```

New or updated recipes:
```
list:
    mbdeploy list

probe:
    mbdeploy probe

deploy *args='':
    mbdeploy deploy {{args}}

build-deploy *args='':
    mbdeploy build && mbdeploy deploy {{args}}
```

Existing `build` and `build-clean` recipes (currently calling `uv run python3 build.py`)
should remain OR be replaced with `mbdeploy build` / `mbdeploy build --clean` —
either form is acceptable as long as the recipes work. Do not break `build` or
`build-clean`.

- [ ] `justfile` contains `mbd-install` recipe calling `pipx install --editable ./mbdeploy`.
- [ ] `just list` calls `mbdeploy list`.
- [ ] `just probe` calls `mbdeploy probe`.
- [ ] `just deploy 1` calls `mbdeploy deploy 1`.
- [ ] `just build-deploy -- 1` calls build then `mbdeploy deploy 1`.
- [ ] `scripts-build` recipe is removed.
- [ ] `just build` and `just build-clean` still work.

### Root `pyproject.toml` Updates

- [ ] `pyocd>=0.44.1` is removed from the `dependencies` list (now owned by `mbdeploy`).
- [ ] `pyserial>=3.5` remains in the `dependencies` list (used by `tests/rogo.py`).

### `README.md` Updates

- [ ] `README.md` documents `pipx install --editable ./mbdeploy` as the install step.
- [ ] `README.md` describes all four `mbdeploy` subcommands with their key flags.
- [ ] `README.md` removes any references to `scripts/deploy.py` or
  `scripts/build_and_deploy.py` as direct invocation targets.
- [ ] Target selectors (enum, name, path, UID) are documented.

### Dockerfile (no change)

- [ ] `Dockerfile` is unchanged — still calls `python3 build.py`.

## Implementation Plan

### Approach

1. Delete files in `scripts/` as listed above.
2. Edit `justfile`: remove `scripts-build`; add `mbd-install`, `list`, `probe`;
   update `deploy` and `build-deploy` bodies.
3. Edit root `pyproject.toml`: remove the `"pyocd>=0.44.1"` line.
4. Edit `README.md`: add/update the deploy tooling section.
5. Run `uv sync` (or `uv lock`) to update `uv.lock` after dropping `pyocd` from
   root deps.

### Files to Delete

```
scripts/deploy.py
scripts/build_and_deploy.py
scripts/build.py
scripts/lib/device_link.py
scripts/lib/known_devices.json   (if present)
scripts/lib/__pycache__/         (if present)
scripts/__pycache__/             (if present)
scripts/lib/                     (rmdir)
scripts/                         (rmdir)
```

### Files to Modify

- `justfile` — per the recipe changes described above.
- `pyproject.toml` (root) — remove `pyocd` from deps.
- `README.md` — update deploy tooling documentation.

### Files NOT to Modify

- `build.py` (root) — unchanged.
- `Dockerfile` — unchanged.
- `tests/rogo.py` — unchanged.
- `uv.lock` — updated automatically by `uv sync`; commit the result.

### Testing Plan

- **Existing tests**: `uv run pytest` — all existing tests should continue to pass.
- **Smoke verification** (with a connected board):
  - `just mbd-install` — should run pipx install without error.
  - `just list` — should print connected board UID and port.
  - `just probe` — should write/update `config/devices.json`.
  - `just deploy 1` — should flash the board.
  - `just build-deploy -- 1` — should build then flash.
- **No-board verification** (always runnable):
  - `mbdeploy --help` still works after justfile changes.
  - `python -c "import pyocd"` from the root venv should FAIL (confirming pyocd
    was removed from root deps); `mbdeploy` from pipx still works (it has its own venv).

### Documentation Updates

`README.md` is updated as part of this ticket's acceptance criteria.
