---
status: in-progress
sprint: '014'
tickets:
- 014-010
---

# Velocity Charting Tool — Real-Time Strip Charts + Phase Plot

## Context

The robot has a ratio PID controller that keeps the two wheel velocities at a commanded ratio. To visually verify it's working, we need a real-time display that shows:
1. Each wheel's velocity over time (strip charts)
2. A velocity-vs-velocity phase plot where a well-tuned controller keeps the trace on a line through the origin regardless of external disturbances (e.g., a finger on a wheel)

The tool streams live telemetry from the robot over USB serial and renders three updating plots in a matplotlib window.

## Library

**matplotlib** — already in the Python ecosystem, supports FuncAnimation for real-time updating plots, works on macOS out of the box. No new heavy dependencies. The smoke test guard (`test_no_matplotlib_after_subpackage_imports`) only applies to the robot_radio package itself; a standalone bench script has no restriction.

Add `matplotlib` to the `calibrate` dependency group in `pyproject.toml` (already the `default-groups` entry, so `uv run python ...` will find it automatically).

## Files to create/modify

- **`pyproject.toml`** — add `"matplotlib>=3.8"` to the `calibrate` dependency group
- **`tests/bench/velocity_chart.py`** — the standalone charting script

## Implementation

### Connection pattern
Follow `tests/calibrate/calibrate_linear.py`: use `SerialConnection` (with `dsrdtr=False` to avoid micro:bit resets) → `NezhaProtocol` → call `robot.stream_drive(speeds, period_ms=40)` which handles watchdog keepalives. Parse TLM frames for `vel = (vL_mmps, vR_mmps)`.

Key imports from the existing host package:
- `from robot_radio.io.serial_conn import SerialConnection`
- `from robot_radio.robot.protocol import NezhaProtocol, parse_tlm`
- `from robot_radio.robot.nezha import Nezha`

### Threading model
A **daemon thread** runs `stream_drive()` and pushes `(t, vL, vR)` tuples into a `queue.Queue`. The **main thread** runs `matplotlib.animation.FuncAnimation` at ~30 Hz, draining the queue into `collections.deque` rolling buffers, then updating plot artists.

### Figure layout
3-panel figure, portrait orientation:

```
┌─────────────────────────────────────┐
│  Left wheel velocity (mm/s)         │  — strip chart, last N seconds
├─────────────────────────────────────┤
│  Right wheel velocity (mm/s)        │  — strip chart, last N seconds
├─────────────────────────────────────┤
│  vR vs vL (phase plot)              │
│                                     │
│  • reference line through origin    │
│    (slope = cmd_vR / cmd_vL)        │
│  • grey trace of recent history     │
│  • large coloured dot at current    │
└─────────────────────────────────────┘
```

### Phase plot detail
- X-axis: vL (mm/s), Y-axis: vR (mm/s)
- **Reference line**: `y = (cmd_vR / cmd_vL) * x`, plotted in dashed blue, extends through the axis range
- **Trace**: last `window_s` seconds of points as a thin grey polyline (or colored by age with alpha decay)
- **Current point**: large red `•` marker at `(vL[-1], vR[-1])`
- When the ratio controller is working, the current point stays near the reference line even when external disturbances (finger braking) push velocities around

### CLI arguments
```
uv run python tests/bench/velocity_chart.py \
    --port /dev/cu.usbmodem2121102 \   # or auto-detect via mbdeploy list
    --speed 200 \                       # mm/s for both wheels (default 200)
    --window 8                          # seconds of rolling history (default 8)
```

If `--port` is omitted, auto-detect the robot port using `list_serial_ports()` (same pattern as calibrate_linear.py).

### Shutdown
`Ctrl-C` stops streaming (`STREAM 0` + `STOP` sent before exit), closes the serial connection, and closes the window.

## Verification

1. `uv add matplotlib` to the calibrate group succeeds
2. `uv run --with pytest python -m pytest -q` — 1042 still pass (no new imports in robot_radio)  
3. Run the script against the live robot:
   ```
   uv run python tests/bench/velocity_chart.py --speed 200
   ```
   - Strip charts show ~200 mm/s horizontal bands for both wheels
   - Phase plot shows the current point near the reference line (slope ≈ 1 for equal speeds)
   - Light finger touch on a wheel: point moves slightly off-line then returns
   - Heavy finger touch: point moves further off-line, recovers when released
   - Ctrl-C exits cleanly (no stuck motors)
