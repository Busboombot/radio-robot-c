---
status: done
sprint: '033'
tickets:
- 033-004
---

# D distance-stop baseline races the encoder reset — D after TURN can complete with zero motion

## Context

Found by the bench-032 diagnosis (`docs/code_review/bench-032-diagnosis.md` §NEW FINDING),
confirmed against the 032 TLM log. A `D` command not preceded by `ZERO enc` can stop on the
FIRST evaluate and travel zero distance, because the distance baseline is snapshotted from STALE
inputs.

`MotionController::beginDistance` (`MotionController.cpp:340-389`) resets the hardware accumulators,
then calls `_activeCmd.start(inputs, now_ms)`, which snapshots `base.enc0Mm = (encLMm + encRMm)/2`
from `state.inputs` — but `state.inputs.encLMm/R` are zeroed only AFTER beginDistance returns, in
`Robot::distanceDrive` (`Robot.cpp:432-441`). So `enc0` captures the PREVIOUS command's encoder
average. Next tick the collect reads the freshly-zeroed hardware (~0), and the DISTANCE stop
(`StopCondition.cpp:131-139`) computes `traveled = |0 - enc0|` = the stale average. If the previous
command left avg-encoder >= targetMm, the stop fires immediately → instant completion, zero motion.

The comment at `MotionController.cpp:382-386` ("the baseline enc0 captured by MotionCommand::start()
will be 0") is wrong. NOTE: sprint 030-001 (N1) added `resetEncoders()` but did NOT fix this
ordering — the snapshot still precedes the input zeroing.

## Log evidence (032)

- sqD2 (`D 300 250 250`, target 250): prior enc from sqT1 = 90,410 → avg **250.0** ≥ 250 → instant
  stop, zero motion. ✓
- sqD4: prior enc from sqT3 = 183,319 → avg **251** ≥ 250 → instant stop. ✓
- sqD3: prior enc from sqT2 = 67,-66 → avg 0.5 → ran normally. ✓
- All seq-3 drives (`ZERO enc` first) → enc0 = 0 → ran normally. ✓

Corollary even when it doesn't instant-fire: any leftover average silently shortens (or, if
negative, lengthens) the next D by that amount — a distance error on every D not preceded by ZERO.

## Fix

Make the baseline snapshot see zeroed inputs: run the full `resetEncoders()` (zeroing
`state.inputs.encLMm/R`) BEFORE `_activeCmd.start()`, or explicitly set `enc0Mm = 0` for D-origin
commands. Fix the stale comment.

## Acceptance

- Sim test: `D` → `TURN` → `D` with NO `ZERO` between; the second D travels the full commanded
  distance (no instant-complete, no shortened travel).
- Re-run bench square over USB serial: all four D legs move.
