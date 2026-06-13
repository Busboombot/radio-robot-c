# Bench-033 validation findings (post-fix)

Date: 2026-06-12. Robot on stand, talking to its own USB serial
(`/dev/cu.usbmodem2121102`) with raw pyserial (DTR asserted). Firmware
`fw=0.20260612.23` (all sprint-033 fixes), verified via `VER` after a `--clean`
flash.

## Result: all sprint-033 fixes validated on hardware (8/8 checks)

`tests/bench/bench_validation_033.py`:

- **033-002** DBG OTOS BENCH 1 → `bench=1` (bench mode engages).
- **033-003** twist non-zero while moving with the real OTOS off-surface:
  driving `T 150 150` → `twist.v` 183→163→145 mm/s; spinning `T 150 -150` →
  `twist.omega` peaks −1335 mrad/s. (In the 032 run twist was 0,0 the entire
  time — encoder velocity now fuses into the EKF unconditionally.)
- **033-004** `D` after `TURN` with no `ZERO enc` between: stale pre-D2 enc was
  224,279, yet D2 traveled 202 mm of 200 commanded — it no longer
  instant-completes.
- Health: encoders balanced (encL 395 vs encR 392 — no right-undercount),
  velocity smooth (max tick-to-tick Δv 160 mm/s), heading bounded (no
  out-of-control spin).

## New findings (NOT sprint-033 fixes — candidates for a follow-up sprint)

### F1 — `DBG OTOS` prints empty floats on hardware (`ideal=,, otos=,, fused=,,`)

On the micro:bit the `DBG OTOS` query replies with **all float fields blank**:

```
ideal=,, otos=,, fused=,, err=,,
OK dbg otos
```

In the host sim the same command prints full numbers (`ideal=0.0,0.0,0.0000 …`),
so `test_dbg_otos_query_returns_pose_fields` passes — the bug is hardware-only.

Root cause: the handler formats the pose with `%f`. CODAL / newlib-nano is built
**without floating-point `printf` support**, so `%f`/`%g` conversions emit
nothing (the surrounding literal text and commas remain, hence `ideal=,,`).
Every other reply that shows numbers uses integers (`PING t=%d`, SNAP
`pose=x_mm,y_mm,h_cdeg`, `twist=v_mmps,omega_mrad`).

Impact: the Bench OTOS device (sprint 031) is readable on hardware only via SNAP
(integer pose/twist), NOT via its dedicated `DBG OTOS` readout. The synthetic
pose itself works — it fuses through the EKF and shows up in SNAP — only the
DBG float print is broken.

Fix (future ticket): format `DBG OTOS` pose/vel/err as scaled integers the way
SNAP does (e.g. mm and milli-radians / centidegrees), or enable float printf in
the CODAL link (costs flash/RAM — integer formatting is the cheaper, consistent
choice). Add an on-target-representative check (the sim can't catch this because
its libc has full `%f`).

### F2 — `robot_radio.SerialConnection(mode="direct")` drops TLM/non-OK lines

`SerialConnection` direct mode only surfaces lines it recognises as command
replies (OK/ERR with corr-id); it silently discards `SNAP`'s `TLM …` frame and
the `ID`/`DEVICE:` banner lines. So a bench harness built on it (033-001's
rewrite) cannot read telemetry — `SNAP` comes back as `[]`.

Workaround used here: raw pyserial reader (`tests/bench/bench_validation_033.py`).

Fix (future ticket): teach `SerialConnection` direct mode to also capture
unsolicited/`TLM` lines (return them under a separate key, e.g. `"telemetry"`),
or add a `read_raw()` path for bench/telemetry use.
