# host_tests/ — layout & conventions

`host_tests/` holds the firmware **host simulation tests** plus the build
infrastructure. Keep the **root clean** — only infrastructure lives here:
`conftest.py`, `CMakeLists.txt`, `firmware.py` (the `Sim` wrapper), `sim_api.cpp`.

Subdirectories:

- `unit/` — the maintained pytest **unit suite** (`test_*.py`). `conftest.py`
  builds `libfirmware_host` once per session and puts this directory on
  `sys.path`, so tests anywhere under `host_tests/` can `from firmware import Sim`.
- `dev/` — **development, exploratory, demo notebooks, and one-off scripts**,
  plus their output artifacts (PNG / CSV / logs). Anything ad-hoc goes HERE.
- `playfield_tour/` — the simulated / bench playfield tour (notebook + driver
  module + deskew assets).
- `build/` — CMake build output (generated; not source).

## RULE

Do **not** drop test files, loose scripts, demo notebooks, or output artifacts
(images, CSVs, logs) in the root of `host_tests/`. A maintained unit test goes in
`unit/`; a one-off / demo / probe is a dev artifact and goes in `dev/`. The root
holds only the infrastructure files listed above.
