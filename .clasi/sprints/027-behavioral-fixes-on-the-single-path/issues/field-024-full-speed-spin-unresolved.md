---
status: in-progress
sprint: '027'
tickets:
- 027-006
---

# Field test of sprint 024: robot still does a full-speed spin (root cause unresolved)

Captured at the close of sprint 024 so the finding isn't lost. The 024 code
(PRE_ROTATE bounding, watchdog re-role, EKF heading fusion, gate recovery, slip)
is implemented and all 1434 host/dev tests pass, but the **on-field behavior is
not fixed** and needs more design before another sprint.

## What was verified on hardware (2026-06-11)
- 024 firmware flashed + booted clean: `GET ekfRHead` is recognized (024-only key);
  OTOS alive; EKF fused pose tracks OTOS at rest.
- Driving `G` to a colored box (`square_run.py --boxes black-N`) produced a
  **full-speed spin** that ended with the robot jammed into the boards. Same class
  of failure the sprint set out to fix.

## What is NOT the cause (verified — do not re-chase these)
- **Encoders are healthy.** `tests/dev/enc_watch.py` was run 3× and both wheels
  count cleanly (e.g. L 0→211, R 0→207; second run L 193→405, R 230→417). The
  `enc=0` seen in the square_run log is NOT a frozen encoder. See
  [[dont-reflexively-blame-frozen-encoders]].
- **PRE_ROTATE termination does stop the motors.** `MotionController.cpp` ~line 839
  (the TIME-net branch of the `!still_running` block) calls
  `_mc.stop(); _bvc.reset(); _mode = IDLE`. So "command ends without stopping
  motors" is not it.

## The real open anomalies (for the design round)
1. **SNAP telemetry reported `enc=0` AND `mode=IDLE` while the robot was physically
   spinning at full speed.** The STREAM/TLM `.enc` path reads fine (enc_watch), so
   this is a SNAP-specific discrepancy — investigate SNAP vs STREAM TLM frame
   building (note 024-005 changed `buildTlmFrame`). `mode=IDLE` while the motors
   spin is the key smell.
2. **The bench program abandons the autonomous `G` without stopping it.**
   `square_run.py` declared "reached" off a (wrong) fused pose and exited; the
   firmware `G` runs autonomously, so the robot kept spinning until manually X'd.
   Either the host must `X` on exit / on its own arrival decision, or the firmware
   pre-rotate genuinely ran away — needs to be disentangled with live ground truth.
3. **Why did the pre-rotate spin at full speed at all?** The G pre-rotate gates on
   the fused heading; characterize the actual fused heading during a hardware
   pre-rotate (with the camera as truth) to see whether the gate ever closes and
   whether the 024 ramp/TIME-net behaved as designed on real hardware.

## Tooling added this session
- `tests/dev/enc_selftest.py` — per-wheel live encoder probe (reads ENC off the
  TLM stream during commanded motion; reports per-wheel and flags the
  jammed-wheel/idle-silence confounds). Use the stand (wheels free).

Related: [[wild-spin-root-cause-prerotate-no-timeout]], the sprint-024 architecture
update, and the sim2real review defects (D8 pursuit, D9 OTOS validity, D10 telemetry).
