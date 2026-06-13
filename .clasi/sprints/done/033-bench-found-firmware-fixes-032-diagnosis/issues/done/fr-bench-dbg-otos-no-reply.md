---
status: done
sprint: '033'
tickets:
- 033-002
---

# DBG OTOS BENCH: "no reply" was wrong transport (resolved); REAL bug = bench mode never engages

## Part A — "no reply on hardware" — RESOLVED (transport, not a firmware bug)

The original symptom (`DBG OTOS BENCH`/`DBG OTOS` silent) was because the 032 bench harness talked
over the RADIO RELAY. DBG descriptors are registered `ForceReply::SERIAL` by design
(`DebugCommandable.h:23-24`, `DebugCommandable.cpp:694-704`): debug replies always go to the robot's
USB serial regardless of the arrival channel. Listening on radio never sees them. Confirmed by the
bench-032 diagnosis (`docs/code_review/bench-032-diagnosis.md` §Finding 2). Verified: over the
robot's USB serial (`SerialConnection(..., mode="direct")`) every DBG command replies normally
(`PING -> OK pong`, `DBG OTOS BENCH 1 -> OK dbg otos bench=...`, `DBG OTOS -> OK dbg otos`).
STAKEHOLDER DIRECTIVE: run bench tests over the robot's USB serial, not the relay. No `ForceReply`
change needed.

## Part B — REAL firmware bug: `DBG OTOS BENCH 1` does not enable bench mode

Tested over USB serial (transport ruled out): `DBG OTOS BENCH 1` (and `DBG OTOS BENCH 1 20 10`)
reply **`OK dbg otos bench=0`** every time — the enable flag never takes. `DBG OTOS BENCH 0` also
replies `bench=0`. So the bench-mode pointer swap never engages, which is why bench mode was NOT
active during the entire 032 run (matches the diagnosis cross-check: twist/ekf_rej both 0 while pose
ran to a phantom 131° — impossible if the always-valid BenchOtosSensor were active).

The handler (`DebugCommandable.cpp handleDbgOtosBench`) parses `enable=atoi(tokens[0])` and calls
`nh->setOtosBench(enable!=0)`, then replies `bench = nh->isBenchMode()?1:0`. `setOtosBench`/
`isBenchMode` (`NezhaHAL.h:63-75`) swap/compare `_otosActive` against
`static_cast<IOtosSensor*>(&_benchOtos)`. Something in this parse→swap→isBenchMode path is not
flipping (or not reading back) the active pointer. Investigate: confirm `enable` is actually 1 at the
handler (token/arg plumbing), that `setOtosBench(true)` assigns `_otosActive=&_benchOtos`, and that
`isBenchMode()`'s pointer comparison holds (watch for cast/const-adjustment or a separate HAL
instance). Add an on-target check (or a host-reachable seam) so this can't silently regress.

## Acceptance

- Over USB serial: `DBG OTOS BENCH 1` → `OK dbg otos bench=1`, `DBG OTOS BENCH 0` → `bench=0`.
- With bench mode ON, `otos()` returns the BenchOtosSensor and `Robot::benchOtosTick` feeds it →
  `DBG OTOS` shows `ideal`/`otos` advancing under commanded motion, and `twist` becomes non-zero
  (also depends on Finding 3 enc-velocity un-gating). Re-verify in the serial bench re-run.

## RESOLUTION (033-002) — root cause was NOT the pointer comparison

Part B's hypothesis (something wrong in `setOtosBench`/`isBenchMode`'s pointer swap) was a red
herring; `NezhaHAL.h:63-75` was correct all along. The real bug was a **C++ union-aliasing clobber
in `parseDbgOtosBench`** (`DebugCommandable.cpp`). `Argument` holds `union { int32_t ival; float
fval; }`. The parser did `args[0].ival = atoi(tokens[0])` and then `args[0].fval = 0.0f` on the SAME
union — `0.0f` is bit-pattern `0x00000000`, so it overwrote `ival` back to 0. The handler therefore
read `enable=0` for every `DBG OTOS BENCH 1` and replied `bench=0`. (The noise float args had the
mirror-image clobber: an `ival = 0` after `fval = <noise>`.) Fix: never write the other union member
after setting one. The toggle now also routes through `Robot::setBenchOtosEnabled` so the HOST sim
can regression-test the enable path (`host_tests/test_dbg_otos_commands.py`,
`test_dbg_otos_bench_enable*`). Hardware-verified over USB serial: `DBG OTOS BENCH 1 → bench=1`,
`DBG OTOS BENCH 1 20 10 → bench=1`, `DBG OTOS BENCH 0 → bench=0`.

Process note: an earlier attempt reached for `objdump` and a "nRF52 pointer-comparison" theory — both
dead ends. The bug was found by reading the parser and adding one `en=%d` probe to the reply.
