# Bench calibration programs

Interactive, camera-validated calibration for the robot (tovez), adapted from
the prior repo's `test/calibrate/`. These are bench tools, not unit tests —
they drive the real robot and read the overhead camera.

## What you need
- Robot powered on, **on the field** (real floor motion), wearing **AprilTag 100**.
- The RADIORELAY plugged in (default `/dev/cu.usbmodem21421302`).
- The overhead **aprilcam** daemon running (AprilTags project) — provides ground truth.
- The line **laser on port 4** (turned on automatically for marking the start).

Run from **this** project. Its venv provides both `pyserial` and the `aprilcam`
camera client — `aprilcam` is declared as the `calibrate` dependency group in
`pyproject.toml` (a default group), so a one-time `uv sync` makes plain
`uv run` work:

```sh
uv run python tests/calibrate/calibrate_linear.py
```

## `calibrate_linear.py` — distance calibration
Closed loop. Each trial drives forward a fixed distance (default **900 mm**),
then calibrates **both** distance estimators against ground truth and pushes
the new values to the robot, so accuracy improves run to run:

- **Encoders** → `mm_per_wheel_deg_left/right` (`SET ml/mr`)
- **OTOS odometer** → linear scalar int8 (`OL`), measured from raw OTOS LSB
  (`OP`, 0.305176 mm/LSB) so it is independent of the encoder calibration

Ground truth is the **tape measure** if you type one each trial, otherwise the
**camera** displacement of tag 100. The camera distance is always shown for
validation, and tape-vs-camera disagreement is flagged.

Controls: **Enter** drives a trial, **q** quits. On quit the final values are
written to `data/robots/tovez.json` (use `--no-write` to skip the file write;
the robot still holds the live values until reboot).

Options: `--distance MM` `--speed MMPS` `--port DEV` `--no-write`
`--field "W H"` (mm; refuses any drive whose predicted end leaves the field —
a guard against driving into a wall).

## Library
`calibrate_linear.py` uses the `robot_radio` library directly:
- `robot_radio.robot.nezha.Nezha` + `NezhaProtocol` — all robot commands
- `robot_radio.config.robot_config.load_robot_config` — reads `data/robots/tovez.json`
- `aprilcam` — overhead camera ground truth (internal `_Cam` helper)
