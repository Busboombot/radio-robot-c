# robot_radio — canonical Python library for the Nezha robot (protocol v2)

`host/robot_radio/` is the **canonical, fully-tested** Python library for all
host-side robot interaction.  Everything that talks to the physical robot goes
through this package.  Protocol v2 is the only supported wire format.

---

## Package layout (layered architecture)

```
robot_radio/
  robot/
    protocol.py        NezhaProtocol — owns the serial port; all v2 wire I/O
    nezha.py           Nezha — high-level driver (motion, sensors, config)
    robot.py           Robot — abstract base class
    nezha_state.py     NezhaState — mutable live-sensor state helper
    nezha_kinematic.py NezhaKinematic — closed-loop kinematic helpers
    clock_sync.py      ClockSync — robot-clock ↔ host-clock alignment
  sensors/
    otos.py            OTOS odometer adapter
    odom_tracker.py    OdomTracker — rolling encoder + OTOS integrator
    cam_tracker.py     CamTracker — overhead AprilTag camera ground-truth
    odometry.py        Odometry — dead-reckoning integrator
    calibration.py     push/pull calibration helpers
    color.py           Color sensor adapter
    motion_monitor.py  MotionMonitor — velocity + stillness detection
  config/
    robot_config.py    load_robot_config / save_robot_config (data/robots/*.json)
  nav/                 Navigator + PoseAlign — high-level navigation
  path/                PathBuilder, BezierPath, CatmullRom, ArcPath, …
  controllers/         PID, LTV, PurePursuit, Stanley — path controllers
  kinematics/          Differential-drive kinematics helpers
  io/
    serial_conn.py     SerialConnection — raw serial wrapper
```

**Dependency direction** (strict, no cycles):

```
controllers / nav / path
        ↓
    kinematics
        ↓
  sensors / config
        ↓
    robot (Nezha / NezhaProtocol)
        ↓
      io (SerialConnection)
```

---

## Running tests

Full library suite (409 tests, ~1 s):

```sh
uv run --with pytest python -m pytest host/tests
```

Full project suite including firmware-logic tests (1012 tests, ~1 s):

```sh
uv run --with pytest python -m pytest -q
```

### pytest scope guardrail

`pyproject.toml` (repo root) pins `testpaths = ["host/tests", "tests"]` and
excludes `vendor/` via `norecursedirs`.  **Do not** run a bare `pytest .` from
the repo root — it would recursively collect `vendor/PythonRobotics` (90+
matplotlib/numpy simulation tests) which can exhaust memory.

---

## Import path

`robot_radio` lives under `host/`.  Scripts that run from the repo root must
add `host` to the Python path:

```python
import sys
sys.path.insert(0, "host")
from robot_radio.robot.nezha import Nezha, NezhaProtocol
from robot_radio import nav, path, controllers, kinematics
```

Smoke-check (exits 0 if all sub-packages import cleanly):

```sh
uv run python -c "import sys; sys.path.insert(0, 'host'); \
    from robot_radio.robot import Nezha, NezhaProtocol; \
    from robot_radio import nav, path, controllers, kinematics"
```

---

## Smooth-driving guidance

### Discrete moves — blocking `T` / `D` (no watchdog)

Use `T` (timed) and `D` (distance) for point-to-point moves.  The firmware
waits for the move to complete, then sends `EVT done T` / `EVT done D`.
The `Nezha` driver blocks until the event arrives — no watchdog is involved,
so there is no risk of a `EVT safety_stop` mid-move.

```python
# Drive 500 mm at 200 mm/s — blocks until complete
proto = NezhaProtocol(SerialConnection("/dev/tty.usbmodem…"))
nezha = Nezha(proto)
nezha.connect()
nezha.speed_for_distance(200, 200, 500)   # (left_mms, right_mms, mm)
```

### Continuous driving — `stream_drive()` keepalive

For sustained or steered motion use `Nezha.stream_drive()` (or the lower-level
`NezhaProtocol.stream_drive()`).  It sends periodic `S` keepalives faster than
the watchdog window, so the motors run smoothly without interruption.

```python
speeds = [200, 200]
for resp in nezha.stream_drive(speeds, duration_s=3.0):
    # steer by mutating speeds[] in place
    speeds[0] = new_left
    speeds[1] = new_right
```

`stream_drive` yields `ParsedResponse` objects (TLM, EVT, …) on every poll
cycle so callers can react to sensor data while driving.

### Firmware watchdog — `sTimeoutMs = 500 ms`

The firmware's streaming-speed watchdog (`sTimeoutMs`) defaults to **500 ms**
(raised from 200 ms in Sprint 013 / ticket T006).  The 500 ms window gives
enough headroom to tolerate relay-link jitter without cutting motors.  The
`stream_drive` keepalive period is 40 ms by default — well inside the window.

To inspect or adjust the value on a live robot:

```
GET sTimeout          # → CFG sTimeout=500
SET sTimeout=500      # restore default
```

---

## Calibration tool

Closed-loop linear calibration (distance, encoders, OTOS):

```sh
uv run python tests/calibrate/calibrate_linear.py
```

Run from the repo root.  The `calibrate` uv dependency group (declared in the
root `pyproject.toml` and included in `default-groups`) provides the
`aprilcam` camera client — a plain `uv run` is sufficient, no `--group` flag
needed.

`calib_common.py` has been **removed** (Sprint 013, T007).  `calibrate_linear.py`
now imports `robot_radio` directly for all robot commands and config I/O.

See `tests/calibrate/README.md` for full hardware prerequisites and options.

---

## Protocol v2 wire format (summary)

| Prefix | Meaning                    | Example                                   |
|--------|----------------------------|-------------------------------------------|
| `OK`   | Command accepted           | `OK pong t=12345`                         |
| `ERR`  | Rejected                   | `ERR badarg missing key`                  |
| `EVT`  | Async event                | `EVT done D`, `EVT safety_stop`           |
| `TLM`  | Telemetry frame            | `TLM t=12345 enc=1024,1019 pose=350,-12,1780` |
| `CFG`  | Config dump                | `CFG ml=0.487 mr=0.481 sTimeout=500`     |
| `ID`   | Identity / capabilities    | `ID model=Nezha2 name=GUTOV`              |

Full command reference is in the module docstring of `robot/protocol.py` and
in `robot/nezha.py`.
