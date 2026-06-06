---
status: in-progress
sprint: '015'
tickets:
- 015-001
- 015-004
---

# Encoder/I2C wedge — line of attack (debug-once-and-for-all plan)

The single-loop firmware intermittently **wedges the encoder reads**: `enc`
freezes at a constant (often `0,0`) while a drive is commanded. It recovers
**only on a micro:bit reset** (DTR pulse from reopening the port, or a reflash),
never on its own. This doc is the plan to find and kill it for good.

---

## 1. The one hard fact that constrains every theory

**A micro:bit reset recovers it; the Nezha is NOT re-initialized by that reset.**
`main()` constructs the `Motor` objects but sends the Nezha no init/reset command.
So whatever is wedged lives on the **micro:bit side** — the nRF52 **TWIM** (I2C
master) peripheral or CODAL driver state — not in the Nezha chip and not a slave
physically holding SDA (a full power-down would be required for that, and is not).

Corollary: the fix is almost certainly **"reinitialize/recover the TWIM in
firmware"** and/or **"stop provoking the TWIM error that wedges it."** We should
be able to recover *without* a reset once we find it.

---

## 2. What we've tried (chronological, with outcomes)

Velocity-source / throb lineage (sprints 008–013):

| # | Change | Sprint | Outcome |
|---|---|---|---|
| 1 | Add chip `readSpeed` (0x47) as velocity source | 008 | Worked in isolation |
| 2 | Fix readSpeed mm/s conversion | 010 | In firmware it stuck ~30 mm/s — I2C interleaving read 0x47 before the chip settled |
| 3 | readSpeed post-write settle 4 ms → 8 ms | 012 | Partial; throb persisted |
| 4 | High-res float encoder differencing | 013 | Velocity noise down (L CV 0.9→0.04) |
| 5 | **Drop per-tick 0x47 readSpeed; encoder-delta only** | 013 | **Throb fixed.** 0x47 disabled at policy level |
| 6 | Isolate control on a dedicated fiber + **busy-wait atomic I2C** (no fiber_sleep mid-transaction) | 013 | Stabilized; proved fibers were interleaving I2C |

Single-loop refactor + the current wedge fight (sprint 014, this session):

| # | Change | Outcome |
|---|---|---|
| 7 | Split-phase encoder request/collect, remove busy-waits | Reverted in practice — per-tick **atomic** read (8 ms busy-wait) restored closed-loop velocity |
| 8 | `begin()` in `main()` after 2.5 s settle; color re-wake each retry | Sensor cold-boot detection fixed |
| 9 | Single cooperative loop (LoopScheduler), abandon fibers | ~42 Hz; all sensors + encoders read together |
| 10 | **`Motor::setSpeed` write-on-change** (don't re-write the Nezha every ~100 Hz tick) | Kept; reduces bus writes |
| 11 | **Stop COASTS via `0x60 speed 0`**, not the `0x5F` "shutdown" command | Kept; matches old TS firmware |
| 12 | `stop()`/`startDrive()` use **cached** enc (`_prevEncL/R`), no atomic reads from the comms path | Kept; removes comms-path reads |
| 13 | **Skip encoder reads while idle** (only read when commanded to drive) | Kept |
| 14 | **Fixed a stale `static` local** in a debug open-loop block that skipped the restart `setSpeed` after a stop | Fixed the "first drive works, second does nothing" reproduction |

---

## 3. What actually worked / what the evidence shows

- **Continuous drive is rock-solid.** Sensors off + open-loop, write-on-change,
  watchdog off: encoders counted 0→2549 over 8 s, steady ~150 mm/s, never froze.
  → The encoder **read path itself is correct**; 100 Hz reads while driving are fine.
- **Headless drive/stop cycling passes.** Real PID + **all sensors on** +
  carriage: 7 consecutive drive→stop→drive cycles all counted; velocity steady
  ~182 mm/s, balanced L/R; line/color varying each cycle. → The single-loop
  architecture and the stop/restart path work under clean serial control.
- **The wedge shows under `velocity_chart.py` (GUI), intermittently.** It does NOT
  reproduce in the headless cycle test. The two differ in exactly two ways:
  1. The GUI's matplotlib loop shares the GIL with the streaming worker; a stall
     slips the keepalive past the firmware S-watchdog → **`fullStop` fires
     mid-stream** (a different stop trigger than a clean idle `STOP`).
  2. The GUI's `SerialConnection` opens with `dtr=False`, so connecting does
     **not** reset the micro:bit → it cannot clear a wedge inherited from a prior
     run, and the wedge then presents as `enc=0` from the very first frame.
- **Sensors-off felt better** (stakeholder observation). With only the Nezha
  (0x10) on the bus there are no other-device transactions between encoder reads.
- **One cause already removed:** the stale-`static` restart bug (#14) — that was
  the "second drive does nothing" symptom, distinct from the freeze-mid-drive.

Net: the encoder read is fine in isolation; the wedge correlates with **(a)** a
`fullStop` during active streaming and/or **(b)** other-device I2C traffic on the
shared bus, and it is **micro:bit-side, reset-recoverable** state.

---

## 4. Key facts about the current I2C implementation (for theories)

- **One shared external bus** `uBit.i2c` (P20/P19), **100 kHz** (default; never
  bumped). Devices: Nezha 0x10, OTOS 0x17, line 0x1A, color 0x43.
- **No mutex / no lock / no critical section** anywhere. Serialization is implicit
  in the cooperative loop order.
- **Every `_i2c.write()/read()` return code is discarded.** I2C errors
  (NACK/timeout) are completely invisible to us today.
- The control task's encoder read is an **atomic** read (`readEncoderAtomic`,
  4 ms busy-wait → write 0x46 [STOP] → 4 ms busy-wait → read 4 B [STOP]). The
  busy-wait holds the CPU, so **no task runs in the gap**; the gap is protected.
- The radio datagram **ISR copies bytes only — it does not touch I2C.** Serial is
  polled, not interrupt-I2C. So no interrupt-context code competes for the bus.
- Per tick the order is: encoder(0x10) → PID → setSpeed(0x10) → comms → drive →
  odometry → otos(0x17, 100 ms gate) → **line(0x1A, every tick) → color(0x43,
  every tick)** → ports → telemetry → idle-sleep → next tick encoder(0x10).
  Line and color are **ungated** (read every iteration).
- CODAL has bus recovery (`waitForStop` → on ERROR: clear ERRORSRC, RESUME, STOP,
  and if still stuck `redirect()` = full re-init incl. 9-clock `clearBus`). We
  never observe whether it fires. `setBusIdlePeriod()` / `minimumBusIdlePeriod`
  exists but is **0** (unused) on this target.

---

## 5. Theories (ranked)

**T1 — Unrecovered TWIM error from a shared-bus sensor read (LEADING).**
A line/color/OTOS read intermittently NACKs or times out. CODAL's auto-recovery
fires but doesn't fully heal before the next 0x10 encoder read, OR the recovery
(RESUME/STOP/redirect) perturbs the Nezha's read window. The TWIM is left in a
state where 0x10 reads return stale/zero until a peripheral reinit (= micro:bit
reset). Fits: sensors-off-helps, reset-recovers, intermittent, error codes unseen.

**T2 — `fullStop` during active streaming wedges it.**
The wedge correlates with the watchdog firing mid-stream (GUI keepalive slip).
`fullStop`→`stop()`→`setSpeed(0)` then resumed streaming may drive a bus/Nezha
sequence the clean idle `STOP` path never hits. The headless test (clean STOPs)
passes; the GUI (watchdog STOPs) wedges.

**T3 — Concurrency / overlapping transactions on the bus.**
Analysis says I2C concurrency is **structurally eliminated today**: the loop is
cooperative, the encoder gap and `waitForStop()` are **non-yielding busy-waits**
(the cooperative scheduler can't switch fibers mid-transaction and won't preempt
a non-yielding fiber), and no ISR/event handler issues I2C (the radio ISR copies
bytes only). So a plain **mutex/lock would be a runtime no-op** — there is no
second context to exclude, and adding one would neither fix the wedge nor tell us
anything. **We do not yet TRUST this analysis, so we will MEASURE it** with a
re-entrancy guard (Phase 1) — if the guard ever trips, the analysis is wrong,
there IS an overlapping caller, and a real lock fixes it. The genuine levers from
the vendor warning (*"serialize all access to 0x10, keep the gap uninterrupted"*)
are **enforced inter-transaction idle/settle gaps** and a **single recovery
owner**, not the exclusion itself.

**T4 — Bus speed / electrical margin.** External bus runs at 100 kHz (good,
conservative). Low prior, but worth confirming we never bumped it and that pull-up
/ open-drain init matches CODAL's `redirect()`.

**T5 — `velocity_chart` artifact only.** The GUI connects without a DTR reset and
can't recover an inherited wedge; some/all "failures" are it reporting a
pre-existing wedge, not causing one. Real but likely secondary.

---

## 6. The plan — debug once and for all

Guiding principle: **make it reproduce deterministically and make the failure
visible from inside the firmware before changing any fix.** No more guessing from
a flaky GUI.

### Phase 0 — Deterministic, headless reproduction harness
- Build `tests/bench/wedge_repro.py`: pure-serial (no matplotlib), drives
  drive→stop→drive cycles with **sensors on**, and on each cycle detects the
  wedge (commanded but `enc` Δ ≈ 0). Add modes to force the suspected triggers:
  `--watchdog-stop` (let the keepalive lapse so the firmware watchdog fires the
  stop, then resume) vs `--clean-stop`; `--no-reset-connect` vs DTR-reset connect.
- Run each mode for many cycles to get a **wedge rate** (e.g. X/100). Goal: a
  config that wedges reliably within ~20 cycles. Without a deterministic repro we
  cannot trust any fix.

### Phase 1 — Make the failure observable (instrument first, fix nothing yet)
- **Re-entrancy GUARD (do this first — answers "is it concurrency?" for certain).**
  Route every I2C access through one `I2CBus` wrapper holding a `bool _inUse`
  flag, set on entry and cleared on exit of each transaction. If a transaction is
  entered while `_inUse` is already set, **record a re-entrancy violation**
  (counter + the address already in flight + the new address). Make the flag
  check/set atomic with a short `target_disable_irq()`/`target_enable_irq()` so
  the guard would correctly catch an **ISR-context** re-entry too. This is a
  diagnostic, not a fix: a plain lock is a no-op here (see T3), but the guard
  turns the concurrency assumption into a measured fact. If it **never trips**
  across a wedge → concurrency is ruled out and we focus on TWIM recovery; if it
  **trips** → there is an overlapping caller and a real lock is warranted.
- **Check every I2C return code.** The same `I2CBus` wrapper captures the CODAL
  status of every read/write; keep a per-device error counter and the last error.
  (Today these are discarded — the single highest-value visibility step.)
- Add a `DBG I2C` serial command that dumps: per-device transaction count, error
  count, last error code, **re-entrancy violation count**, and a "consecutive
  identical encoder reads while PWM≠0" stuck-counter.
- Add firmware-side wedge detection: if N consecutive 0x10 reads return the same
  value while commanded to move, emit `EVT enc_wedged` over serial with the bus
  error + re-entrancy stats. Now the harness can correlate the wedge with real
  bus errors and/or re-entrancy.

### Phase 2 — Discriminate the theories (one variable at a time, measured)
- **T1 (sensor error → wedge):** with the harness + instrumentation, compare
  wedge rate and error counts for: (a) all sensors on, (b) line+color gated to
  100 ms, (c) sensors off. If wedges track sensor error counts and vanish with
  sensors off → T1 confirmed.
- **T2 (fullStop mid-stream):** compare `--watchdog-stop` vs `--clean-stop` wedge
  rates. If watchdog-stop wedges and clean-stop doesn't → T2 confirmed; inspect
  the `fullStop`/restart bus sequence.
- **T1/T2 recovery test (decisive):** on `EVT enc_wedged`, have the firmware call
  an **explicit TWIM recovery** (re-init: disable→clearBus 9-clock→re-enable, the
  CODAL `redirect()` equivalent) and check whether encoder reads resume **without
  a micro:bit reset**. If recovery heals it → the root cause is an unrecovered
  TWIM error and the production fix is "detect + recover" (and/or prevent the
  triggering error).
- **T3 (idle gap / lock):** add `setBusIdlePeriod(...)` and/or an `I2CBus` that
  enforces a minimum idle gap before any 0x10 read; measure wedge-rate change.

### Phase 3 — Implement the indicated fix, then prove it
Likely outcome is a combination:
- An **`I2CBus` wrapper** that all devices go through: serializes access, enforces
  an inter-transaction idle/settle window (esp. before Nezha reads), checks return
  codes, and runs a **deterministic bus recovery** on error (CODAL-style
  clearBus + re-init) instead of silently wedging — this is the concrete form of
  the stakeholder's "lock," and it makes the wedge self-healing even if it occurs.
- Possibly gate line/color reads off the every-tick path (also raises PID rate
  ~42→~75 Hz) to reduce shared-bus pressure.
- Harden `velocity_chart`/tools: optional DTR reset on connect so a tool can clear
  an inherited wedge; the firmware widening (`SET sTimeout`) already lets the GUI
  keepalive tolerate GIL stalls.
- **Acceptance:** the Phase 0 harness runs its worst-case mode (watchdog-stops,
  sensors on, no-reset connect) for ≥100 cycles with **zero** wedges that require
  a micro:bit reset, and `velocity_chart.py` drives→stops→drives repeatedly for
  several minutes with no freeze. Any wedge that does occur self-heals via the
  firmware recovery and is reported as `EVT enc_wedged` + recovered.

### What "done" looks like
We can articulate the exact bus error that caused it (from real error codes), the
firmware recovers from it without a reset, and the harness proves it across the
trigger that used to wedge it.
