---
id: '004'
title: Python test harness + host test suite
status: done
use-cases:
- SUC-004
depends-on:
- 020-003
github-issue: ''
issue: plan-sprint-020-firmware-host-testing.md
completes_issue: true
---

# Python test harness + host test suite

## Description

Create the Python ctypes loader (`host_tests/firmware.py`), pytest fixtures
(`host_tests/conftest.py`), and four layered test files for the host simulation library
produced by ticket 020-003. Tests are organized bottom-up: MockHAL physics first, then
PID convergence, then motion state machines, then command routing.

Each test creates a `Sim` context manager, advances time via `tick_for()`, and sends
commands via `send_command()`. No hardware required.

## Acceptance Criteria

- [x] `host_tests/firmware.py` created: ctypes loader declaring argtypes/restypes for all `sim_*` functions; `Sim` class with `__enter__`/`__exit__`; `tick_for(total_ms, step_ms=24)` helper; `send_command(line)` helper that splits reply lines.
- [x] `host_tests/conftest.py` created: `build_lib` session-scoped autouse fixture that runs `cmake --build` if library is missing or sources are newer; `sim` function-scoped fixture that creates and destroys a `Sim`.
- [x] `host_tests/test_mock_hal.py` passes:
  - At 100% speed, encoder grows >= 70% of `kNominalMaxMms` after 1 s of ticks (threshold lowered from 80% to account for PID ramp-up time).
  - At 0% speed, encoder is stable.
  - At -100% speed, encoder decreases.
  - `sim_set_enc_l()` injection: encoder read-back matches injected value after one tick.
- [x] `host_tests/test_motor_controller.py` passes:
  - PWM is nonzero at 200 mm/s target after 2 s settling.
  - Encoder grows at least 100 mm over 2 s at 200 mm/s.
  - Integral windup: force encoder to zero for 2 s at high target — integrator clamps, PWM stays in [-100, 100] range.
  - Stop: after `X` (cancel) command, PWM goes to 0 within one tick.
- [x] `host_tests/test_motion_controller.py` passes:
  - D command 500 mm: tick 10 s, enc_l + enc_r >= 800 mm (sum ~1000 mm), EVT done D received.
  - VW command: encoder grows over 2.4 s at 200 mm/s.
  - VW keepalive timeout: no keepalive for > sTimeoutMs (500 ms) — motors stop (EVT safety_stop received).
- [x] `host_tests/test_command_processor.py` passes:
  - `PING` → reply contains `OK` and `t=` timestamp.
  - `HELLO` → reply contains `DEVICE` and `NEZHA2`.
  - Unknown verb → reply contains `ERR`.
  - `VW 200 0` then tick 2 s → `GET VEL` shows positive velocity.
  - `SET vel.kP=2.0` → `GET vel.kP` returns 2.0.
- [x] `uv run --with pytest python -m pytest host_tests/ -v` passes all 22 tests.
- [x] `uv run --with pytest python -m pytest` (existing tests) still passes (440 host + 902 tests/dev = 1342 total).

## Implementation Plan

### Approach

Write `firmware.py` first (loader + Sim class). Write `conftest.py` (build fixture +
sim fixture). Write each test file incrementally, running after each to catch issues
early. Do not proceed to the next test file until the previous one passes.

### Files to Create

- `host_tests/firmware.py`
- `host_tests/conftest.py`
- `host_tests/test_mock_hal.py`
- `host_tests/test_motor_controller.py`
- `host_tests/test_motion_controller.py`
- `host_tests/test_command_processor.py`

### firmware.py structure

```python
import ctypes, pathlib, subprocess

LIB_PATH = pathlib.Path(__file__).parent / 'build' / 'libfirmware_host.dylib'

class Sim:
    def __init__(self):
        self._lib = ctypes.CDLL(str(LIB_PATH))
        self._declare_types()
        self._h = self._lib.sim_create()
    def __enter__(self): return self
    def __exit__(self, *_): self._lib.sim_destroy(self._h)
    def tick_for(self, total_ms, step_ms=24):
        t = 0
        while t < total_ms:
            self._lib.sim_tick(self._h, ctypes.c_uint32(t))
            t += step_ms
    def send_command(self, line):
        buf = ctypes.create_string_buffer(512)
        n = self._lib.sim_command(self._h, line.encode(), buf, 512)
        return buf.value[:n].decode().splitlines()
```

### conftest.py build fixture

```python
import subprocess, pathlib, pytest

BUILD_DIR = pathlib.Path(__file__).parent / 'build'

@pytest.fixture(scope='session', autouse=True)
def build_lib():
    BUILD_DIR.mkdir(exist_ok=True)
    subprocess.run(['cmake', '-S', '.', '-B', str(BUILD_DIR)], check=True, cwd=...)
    subprocess.run(['cmake', '--build', str(BUILD_DIR)], check=True)
```

### Testing Plan

1. `uv run --with pytest python -m pytest host_tests/ -v` — all 4 test files pass.
2. `uv run --with pytest python -m pytest` — existing tests unaffected.

### Notes

- `test_motion_controller.py` VW keepalive test depends on Phase B changes (system
  watchdog). For Phase A, write the test as `xfail` and remove the marker in the Phase B
  ticket that implements the watchdog.
- On Linux, library extension is `.so`; on macOS, `.dylib`. The conftest build fixture
  should detect the platform and set the correct path in `firmware.py`.
- `sTimeoutMs` for the watchdog test should match the default config value. Read it via
  `GET sTimeout` from the sim if the value is not a compile-time constant.
- Open Question 2 from architecture: noise model. MockMotor has no noise by default in
  ticket 020-002. Tests should not depend on noise. The `sim_set_motor_noise` function
  from Open Question 2 is deferred — not needed for deterministic tests.
