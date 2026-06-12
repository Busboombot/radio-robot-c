---
status: pending
resolution: refuted
---

# Bench finding — "right encoder under-counts" — REFUTED (was a command-format misread)

## Correction (2026-06-12)

**This finding was WRONG.** The "right encoder under-counts" claim was an artifact of
commanding UNEQUAL wheel speeds, not an encoder fault. Re-tested directly with
`tests/bench/enc_balance_test.py` driving TRULY EQUAL wheel speeds:

- `D 200 200 300` / `D 400 400 600` (left == right), 12 drives across two speeds:
  encR/encL ratio = **0.83–1.00 — balanced** every time.

Root cause of the mistake: the firmware `D` command is **`D <leftSpeed> <rightSpeed> <distance>`**
(confirmed in `source/app/MotionCommandHandlers.cpp` parseD: tokens[0]=left, tokens[1]=right).
My original "equal-wheel" drives (`D 250 150 150`, `D 600 400 400`) actually commanded the LEFT
wheel faster than the right (250 vs 150, 600 vs 400), so the left encoder counting more was
expected and correct — not a deficit. This is the "don't reflexively blame the encoders" trap
(project memory): I named the encoder before verifying with an equal-wheel test.

## Residual (separate, real) observation — low absolute travel

On the re-test the absolute counts were tiny (~20 mm) regardless of commanded distance, even at
400 mm/s — whereas early in the session (fresh) the robot drove ~500 mm (`D 500 500 500` -> enc
502,498). Most likely a **drained motor battery** after hours of bench driving, NOT an encoder or
firmware fault. To rule out a drive-distance bug: recharge the motor battery and re-run
`tests/bench/enc_balance_test.py` — if equal-wheel drives reach ~the commanded distance with
balanced L/R, there is nothing here. The single transient `EVT enc_wedged wheel=R` seen earlier
may simply be this low/erratic counting under a weak battery.

## How to reproduce / verify

`uv run python tests/bench/enc_balance_test.py [--speed N --dist N]`
(robot on a stand, relay USB plugged in). Healthy = encR ~= encL on equal-wheel drives.

## Disposition

Refuted as an encoder fault. Keep open only to confirm the low-travel observation against a
freshly-charged battery; if travel is normal when charged, close as not-a-bug.
