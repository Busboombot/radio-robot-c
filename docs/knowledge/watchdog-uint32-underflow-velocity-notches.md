# The S-watchdog uint32 underflow — spurious safety_stops / velocity "notches"

**Date:** 2026-06-06  **Sprint:** 015  **Status:** FIXED

This was the root cause of the long-running "the robot keeps stopping / the
velocity chart is jumpy / the watchdog fires even though I'm sending keepalives"
saga. It is a one-line class of bug and it masqueraded as a dozen different
problems.

## Symptom

- Streaming drive (`S` keepalives) showed periodic **total momentary stops** —
  the wheels stuttered; on a velocity chart every stop is a downward "V-notch".
- `EVT safety_stop` fired repeatedly (every few hundred ms) **even though**:
  - `SET sTimeout=10000` was confirmed live via `GET` (`CFG sTimeout=10000`), and
  - keepalive `S` commands were going out every 100–150 ms (verified: a keepalive
    was sent only 5–24 ms before each fire).
- It got **worse at higher stream/keepalive rates** (STREAM 50 was riddled with
  it; STREAM 100 looked almost clean), which made it feel random and timing/
  load dependent.

## Root cause

The S-mode watchdog computed elapsed time as an **unsigned** subtraction:

```cpp
if ((now_ms - _lastSMs) > (uint32_t)_cfg.sTimeoutMs) { fullStop(); emitEvt("safety_stop"); }
```

`now_ms` is sampled by the scheduler at the **top** of the loop iteration.
The keepalive `S` is processed slightly **later in the same iteration** (comms-in
runs after the control task), and `beginStream()` sets `_lastSMs` from a **fresh**
`systemTime()`. So `_lastSMs` can be **1 ms greater than `now_ms`**.

When `_lastSMs > now_ms`, `now_ms - _lastSMs` **underflows uint32** to ~4.29e9,
which is `> sTimeoutMs` for any sane timeout → `safety_stop` fires. The faster the
keepalive, the more often an `S` lands in that sub-millisecond window → more fires.

On-hardware proof (instrumented EVT): `now=17937 last=17938 dt=4294967…`.

## Fix

Wraparound/ordering-safe **signed** delta — a small negative reads as "~0 ms
elapsed" instead of "4 billion ms":

```cpp
int32_t dt = (int32_t)(now_ms - _lastSMs);
if (dt > (int32_t)_cfg.sTimeoutMs) { ... }
```

See `source/control/DriveController.cpp` (driveAdvance, S-mode watchdog).

## Why it hid for so long

- It looked like an *encoder wedge* (the wheels stop, enc stops changing).
- It looked like a *keepalive/serial* problem (fires despite keepalives).
- It looked *load/rate dependent* (worse at higher rates).
- A blocking serial TX (separate bug, also fixed — see below) compounded it by
  occasionally stalling the loop, which is a *different* way to starve the
  keepalive. Fixing the TX reduced the notches but did not remove them; only the
  signed-delta fix removed them.

## General lesson

**Never compare two `uint32` millisecond timestamps with a plain subtraction
unless you are certain the first is ≥ the second.** Across a cooperative loop,
"now" and "last-event" can be sampled at slightly different points and invert.
Always use a signed delta: `(int32_t)(a - b)`. The same pattern lurks in any
`(now - lastX) > period` check (task scheduling `due()`, velocity dt, etc.) — a
backwards step there underflows to "forever ago / huge elapsed".

## Companion fixes in the same sprint (all in the "rock solid" result)

- **Serial TX is non-blocking (ASYNC).** `SerialPort::send` used CODAL's default
  `SYNC_SLEEP`, which fiber-sleeps the cooperative loop when the 255-byte TX
  buffer is full (slow/absent reader) — stalling control and, in the extreme,
  hard-hanging the firmware. `ASYNC` drops a frame under flood instead of
  stalling. See `source/hal/SerialPort.cpp`.
- **Velocity outlier rejection + EMA.** Occasional corrupt encoder reads produced
  huge bogus velocity spikes (tens of thousands of mm/s); reject samples beyond a
  plausible bound and EMA-smooth the rest. See `MotorController::controlTick`.
- **Slowest-wheel cross-coupling** (`SET sync`): the wheel achieving more of its
  target is slaved to the slower wheel's actual speed at the commanded ratio, with
  a deadband so light touches are absorbed. Setpoint-based (computed before the
  per-wheel PID) so the wheels don't fight.

Result: free-running velocity CV ~0.013 (was ~0.12 with garbage spikes), zero
spurious safety_stops, coupling holds the phase-plot trace on the ratio diagonal.
