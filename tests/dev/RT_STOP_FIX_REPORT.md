# Turn / Runaway Fix — Overnight Report (2026-06-09)

## TL;DR
`rogo turn N` now works: the robot computes the turn on-board and **stops itself**.
The runaway (spins forever / "400°") is fixed and tested every way I could think of.
Camera-verified accuracy is within ~1–3° on small/medium turns, ~3.3% slip on a full
360°. **All changes are flashed but UNCOMMITTED on `master`.**

## Root cause of the runaway (the real one)
When any on-robot motion command finished, `MotionController::driveAdvance()` set the
mode to IDLE but **never zeroed the motors**. The last wheel-velocity target stayed in
place and the motor PID kept driving it **forever**. `rogo turn` correctly received
`EVT done RT`, but the wheels never stopped.

- This is why hard `X`/`STOP` always worked (they call `_mc.stop()`), but a command
  finishing on its *own* stop condition did not.
- It did **not** reproduce in the simulator — the MockMotor coasts to zero on its own,
  while real motors hold their PWM. My first sim test also masked it by sending an
  explicit `X` after each turn. Lesson logged: always measure stop **without** a
  trailing X.

## The fix
`source/control/MotionController.cpp`, completion branch (~line 661):
```cpp
if (!still_running) {
    _mc.stop();      // zero tgtLMms/tgtRMms + setSpeed(0) — same as the X verb
    _bvc.reset();
    _mode = DriveMode::IDLE;
    ...
}
```

## The turn command (`RT`) — already in place from earlier tonight
- New firmware verb `RT <centi-degrees>`: a RELATIVE spin computed on-robot from the
  encoder arc, stopped on the encoder **differential** (`StopCondition::Kind::ROTATION`
  + `MotionBaseline.encDiff0Mm` + `makeRotationStop` + `beginRotation`).
- No OTOS, no heading odometry, no host loop. SOFT stop at 100°/s with an 8 mm
  coast-anticipation, plus a time-bound stop as a runaway backstop.
- `rogo turn` is now a thin client: it sends `RT <cdeg>` and waits for `EVT done RT`.
  It does **not** send `X`.

## Testing done
- **Sim stop-matrix** (`tests/dev/sim_stop_matrix.py`): 15 scenarios = every start
  command (T/D/TURN/RT/S/VW) × every stop path (natural, X, X soft, STOP, mid-command
  interrupt). Reads encoders at stop / +300ms / after the soft-ramp settle.
  → **ALL STOPPED.** (Confirmed the test catches the bug by temporarily reverting the fix.)
- **Sim RT accuracy, honest (no masking X)** (`tests/dev/sim_rt_verify.py`): vs
  ExactPoseTracker ground truth → ±2.2° and **creep = 0.0°** (stops itself).
- **Full firmware suite**: `uv run --with pytest python -m pytest host_tests/` →
  **64 passed.**
- **Hardware stop-matrix** (`tests/dev/hw_stop_matrix.py`): 10 scenarios, streaming
  encoders, post-stop growth must be ~0 → **ALL STOPPED** (0.0 mm growth every row).
- **Camera accuracy on the field** (AprilTag 100, ground truth):
  | cmd | physical | err |
  |-----|----------|-----|
  | +90 | 88.8° | −1.2° |
  | −90 | 89.1° | −0.9° |
  | +45 | 43.5° | −1.5° |
  | +135 | ~131.8° | −3.2° |
  | +360 | ~348° (×2 runs) | ~−12° (3.3% slip) |
  - **Zero creep** confirmed after each (e.g. 0.1° drift over 86 s after a 360).
  - +90 then −90 returned to within 0.1° of start.
  - The **rapid-fire sequence** (90/180/−90/360/90 back-to-back — the original runaway
    trigger) ended **stopped**.

## Known issues / recommendations for the morning
1. **Commit the work.** Firmware (`MotionController`, `MotionCommand`, `StopCondition`,
   `Robot.cpp` HELP) + host (`cli.py cmd_turn`) + tests are all uncommitted on `master`.
2. **Slip calibration (optional polish).** ~3.3% rotational slip at 360° (encoders
   can't see slip). Best fixed with a config rotation-gain (~1.03) from a clean
   multi-trial camera calibration *with you present* — I left it at 1.0 so the sim
   stays honest rather than over-fit noisy night data.
3. **Encoder zeroing between turns is unreliable** — `proto.zero_encoders()` didn't
   take (+90 and −90 read identical encoders). Use the camera for turn accuracy, not
   between-turn encoder reads. Worth investigating the EZ/zero path separately.
4. **Transient serial glitch**: one turn in the rapid sequence threw
   `SerialException: device reports readiness to read but returned no data` (USB/relay
   comms, not firmware). The other four were fine. Watch for it.

## New test harnesses (all under tests/dev/)
- `sim_stop_matrix.py` — start×stop matrix in sim (deterministic regression).
- `hw_stop_matrix.py` — same on hardware via streaming encoders (safety-X bracketed).
- `sim_rt_verify.py` — RT accuracy vs sim ground truth, honest (no masking X).
- `hw_rt_accuracy.py` — encoder-based RT accuracy (note: zeroing caveat above).
- `eturn.py` — host-side encoder-distance turn (superseded by firmware RT; kept for ref).
- `i2c_diag.py` — proves the encoder bus is healthy during a drive (0 I2C errors).
