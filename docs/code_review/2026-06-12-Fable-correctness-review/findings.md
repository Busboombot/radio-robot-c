# Source Code Correctness Review — Round 2

Date: 2026-06-12
Scope: read-only review of firmware under `source/`, plus `host_tests/sim_api.cpp` for sim/real parity. Follow-up to `docs/code_review/2026-06-11-Fable-s2p-review/` and the improvement plan (D1–D12, P0–P3).

Method: full read of the active runtime path (`main.cpp` → `LoopScheduler::run_blocks()` → `loopTickOnce()`), all of `control/`, `app/`, `robot/`, the OTOS/serial/radio/I2C HAL, and the sim entry points. No build or tests were run.

## Status of the previous round's findings

| Prior finding | Status |
| --- | --- |
| High: T/D deadlines not rollover-safe | **Fixed.** TIME stops use signed elapsed (`StopCondition::evaluate` TIME case); soft-stop deadline and the system watchdog use the same pattern. |
| High: encoder/pose reset desync | **Partially fixed.** `Odometry::setPose()` now re-baselines `_prevEncL/R` (camera fixes are clean). But the D-command and `ZERO enc` paths still desynchronize — see N1. |
| High: SET writes invalid live config | **Fixed in structure** (typed strtof/strtol parsing, candidate copy, atomic commit, `validateConfig`). Gaps remain in coverage — see N6. |
| Medium: sensor reads discard status, sticky valid | **Partially fixed.** OTOS now has a STATUS gate, `lastReadOk`, and `EVT otos lost`. Line/color `valid` is still sticky and TLM still checks only the bit (N8); OTOS read failure can still feed zeros for one tick (N9). |
| Medium: obsolete scheduler/control paths | **Mostly fixed.** `run_tasks`/`run_all` are gone; `tickOnce` is shared with the sim. Residue: vestigial `RatioPidController`, `PID_BYPASS`, `Odometry::update()`, dead `DriveMode::TIMED` (N13). |

The architecture is markedly better than last round: the shared `loopTickOnce()` kills the hand-mirrored sim loop, every self-terminating verb carries a TIME net, the EKF heading fusion and gate-recovery work, and the watchdog/keepalive split matches the plan. The new findings below are mostly seams between subsystems rather than broken algorithms.

---

## New findings

### N1 (High): D command and `ZERO enc` still corrupt the world pose / EKF

`MotionController::beginDistance()` calls `_mc.resetEncoderAccumulators()` (`MotionController.cpp:306`) and `Robot::distanceDrive()` zeroes `state.inputs.encLMm/R` (`Robot.cpp:318-319`). Neither path re-baselines `Odometry::_prevEncL/_prevEncR`. On the same tick (queue dispatch runs before odometry in `loopTickOnce`), `Odometry::predict()` computes `dL = 0 − _prevEncL` (`Odometry.cpp:40-43`) — a large negative delta equal to all travel since the last encoder reset — and feeds it straight into the pose integration and `EKF::predict()`. There is no gate on the predict path.

Effect: every `D` command teleports the fused pose backward by the previous segment's length. With OTOS fusion live, the Mahalanobis gate rejects for 10 cycles and the P-inflation rebaseline snaps back (~1 s at `lagOtosMs`=100), during which a queued `G` drives toward a garbage world frame and `ekf_rej` climbs. With OTOS invalid or disabled, the corruption is permanent until `SI`. This also silently degrades the heading: `dTheta` picks up the differential of the stale baselines.

`ZERO enc` (`Robot.cpp:752`) is worse: it resets hardware accumulators and MotorController baselines but leaves `state.inputs.encLMm/R` stale, so the outlier filter in `controlCollectSplitPhase()` rejects every read (delta ≈ −prior travel) until the fresh accumulator climbs back to the stale value — frozen encoders, velocity PID windup, and the same odometry jump once reads resume. This is exactly the failure mode the comment above `distanceDrive()` describes; `ZERO enc` never got the workaround.

Fix: one robot-level `resetEncoders()` that atomically resets hardware accumulators, MotorController baselines, `state.inputs.encLMm/R`, and re-baselines `Odometry::_prevEncL/R` (without touching pose); call it from both `distanceDrive()` and `handleZero`. This was the "minimal correction" in last round's finding; only the `setPose` third of it landed.

### N2 (High): firmware no longer runs the queue path — main.cpp Phase 3 wipes `cmd._queue`

`LoopScheduler`'s constructor wires `_cmd.setQueue(&_queue)` (`LoopScheduler.cpp:108`). Then main.cpp Phase 3 reassigns the processor: `cmd = CommandProcessor(robot.buildCommandTable(&dbgCmd, &sched))` (`main.cpp:215`). The implicit move-assign copies the temporary's `_queue == nullptr`, so on entry to `run_blocks()` the queue is detached and `process()` dispatches every inbound command immediately from `runCommsIn()` — the pre-026 path. `run_test()` knows about this trap and re-wires (`LoopScheduler.cpp:139`, with a comment naming the bug); `run_blocks()` does not.

Two consequences:

1. **Sim/real split, inverted.** `sim_api.cpp` wires the queue and tests the queue path; the firmware runs the immediate path. P1.3's whole point was one dispatch story. (Motion still works because `robot.setMotionQueue(&_queue)` survives the reassignment, so converter VW pushes drain via `dequeueOne` — by accident, the converters are the only thing using the queue on hardware.)
2. **Mid-session mode flip.** The watchdog/halt emergency path in `loopTickOnce()` does `cmd.setQueue(nullptr); cmd.process("X", …); cmd.setQueue(&queue)` (`LoopTickOnce.cpp:61-63, 76-83`). The restore *arms* the queue that was never armed. After the first safety stop or halt, the firmware permanently switches to queued dispatch (1 command/tick drain, overflow possible, SNAP staleness semantics change). Behavior now depends on whether a safety stop has ever fired.

Fix: re-wire `_cmd.setQueue(&_queue)` at the top of `run_blocks()` (one line, mirroring `run_test()`), or give `CommandProcessor` an assignment operator that preserves wiring. Then the `setQueue(nullptr)/restore` dance in `loopTickOnce` is consistent.

### N3 (High): TLM emit can call a null or mismatched reply function

`Robot::telemetryEmit()` calls `fn(tlmBuf, ctx)` with no null check (`Robot.cpp:448`), and `loopTickOnce` invokes it whenever `cfg.tlmPeriodMs > 0` (`LoopTickOnce.cpp:130-134`), passing `ts.activeTlmFn` and `ts.activeCtx`.

1. **Null call:** `_tlmBoundFn` stays nullptr until a STREAM command binds the channel. But `tlmPeriodMs` is also settable via `SET tlmPeriod=100` (registry key `"tlmPeriod"`, `ConfigRegistry.cpp:81`), which does not bind. `SET tlmPeriod=100` with no prior STREAM → null function-pointer call on the next TLM tick → HardFault. The header comment ("nullptr means TLM is suppressed", `Robot.h:148-149`) describes a guard that nothing implements.
2. **Fn/ctx mismatch:** the D10 binding fix binds the *function* (`runCommsIn` derives `_tlmBoundFn` from `_tlmBoundCtx`, `LoopScheduler.cpp:80-88`) but telemetry is emitted with `ts.activeCtx` — the channel of the **last command received**, not the bound stream channel (`LoopTickOnce.cpp:132`). STREAM bound over serial + any later radio command → `serialReplyTlm(msg, &radio)` casts a `Radio*` to `SerialPort*` and calls `sendReliable` on it. Undefined behavior; mixed serial+radio operation is the normal field setup.

Fix: pass `robot._tlmBoundCtx` (the bound channel ctx) together with `_tlmBoundFn`, and guard `fn == nullptr` in `telemetryEmit()` (or refuse `SET tlmPeriod` and funnel through STREAM).

### N4 (Medium-High): `S` (and `_VW`) during an active MotionCommand leaves a zombie supervisor

`beginVelocity/beginArc/beginTurn/beginRotation/beginGoTo` all cancel an active `MotionCommand` first. `beginStream()` (`MotionController.cpp:148-172`) and `beginRawVelocity()` do not. An `S` issued while TURN/G/T/D is active (the queue path routes it through `handleVW`'s `stream=1` branch → `beginStream`):

- seeds the BVC mid-motion (instant jump — the "fast spin signature" the plan worked to remove),
- leaves `_activeCmd` running, so the old command's stop conditions keep evaluating against the new stream — when its TIME/HEADING/POSITION stop fires, `driveAdvance` soft-stops the robot and emits the old command's `EVT done`, silently killing the stream,
- and the old command never gets an `EVT cancelled`.

This is the same defect class as D6, one layer down: the origin guard protects plain `VW` keepalives but `S` bypasses it. P1.1's own verify scenario ("start TURN, inject `S 0 0` mid-turn → TURN must complete") fails on this code: the S retargets the BVC to zero and the TURN dies on its TIME net.

Fix: in `beginStream()` (and `beginRawVelocity()`), cancel any active command first — same three lines the other begin*() entry points use. Decide explicitly whether `S` should instead be rejected/busy-replied while a self-terminating command runs.

### N5 (Medium): `beginTimed`/`beginDistance` skip the cancel-if-active guard — preempted command vanishes without a terminal event

`beginTimed()` (`MotionController.cpp:257`) and `beginDistance()` (`:294`) go straight to `configure()`, which silently resets the previous command's reply sink. Every other verb emits `EVT cancelled` for the preempted command. A host awaiting `EVT done G` that issues a `T` will never get any terminal event for the G — the exact "EVT done never arrived" class the improvement plan attributed to host-side buffer bugs. Two-line fix per method; also restores wire-contract consistency.

### N6 (Medium): config validation gaps — `aDecel`/`aMax`/`vBodyMax`/`yawRateMax`/`yawAccMax`/`sTimeoutMs` unchecked

`validateConfig()` (`ConfigRegistry.cpp:239-258`) checks tw, ctrlPeriod, vWheelMax/steerHeadroom, rotSlip only.

- `SET aDecel=-100`: trapezoid step `dv_max` goes negative and `approach()` moves *away* from the target each tick (`BodyVelocityController.cpp:84-85`) — runaway; the decel caps compute `sqrtf(negative)` → NaN in PURSUE/D hooks (NaN comparisons silently disable the caps).
- `SET aMax=0` / `yawAccMax=0`: BVC can never leave zero; every motion verb stalls until its TIME net (looks like a dead robot).
- `SET sTimeout=0` (or negative): watchdog compare `wdDelta > (int32_t)cfg.sTimeoutMs` fires every tick once armed — X storm.
- `SET vBodyMax=0`, `yawRateMax=0`: all targets clamp to zero.

Add `> 0` checks for the rate/accel family and `sTimeoutMs >= some floor` to `validateConfig`. (Also note the asymmetry: `effectiveSlip()` accepts 0 as "unset → 1.0" but `validateConfig` rejects `rotSlip=0`, so a host can't restore the documented "unset" state.)

### N7 (Medium): queue-mode command loss is silent — `push_back`/`push_front` failures ignored

`CommandProcessor::dispatchTable()` ignores the `_queue->push_back()` return (`CommandProcessor.cpp:148`), and all seven converters ignore `pushVW()` failure (`MotionCommandHandlers.cpp:247` etc.). Capacity is 4 (`CommandQueue.h:18`) and the drain rate is one per ~10–25 ms tick. A 5-line burst from a host script loses line 5 with no ERR — the host just times out. Worse for converters: the converter has already replied `OK drive …`, so a dropped VW means the host believes the motion started. Reply `ERR busy`/`ERR full` on enqueue failure (and for converters, suppress the early OK or emit a follow-up ERR). Today this bites the sim and the post-first-safety-stop firmware (see N2); once N2 is fixed it bites all hardware traffic.

### N8 (Medium): line/color validity still sticky; TLM publishes stale sensor data forever

`lineRead()`/`colorRead()` set `valid = true` on the first success and never clear or age it (`Robot.cpp:263-286`); `buildTlmFrame` gates on the bit alone (`Robot.cpp:339-342`). A line sensor that wedges after boot keeps publishing its last values indefinitely. The freshness fields (`lastUpdMs`, `lagMs`) exist and are maintained — they're just never consulted. Same for the raw `otos=` TLM field, which keeps emitting the last-good pose while `otos.valid` is false. Carried over from last round's Medium; only the OTOS fusion path was fixed. Use `now − lastUpdMs <= 2×lagMs` in the TLM gates (cheap, fields already there).

### N9 (Medium): OTOS validity gate is one tick stale — a failed burst read still fuses zeros that tick

`Robot::otosCorrect()` checks `otos.lastReadOk()` *before* this tick's reads (`Robot.cpp:209`), but `_lastReadOk` is updated by `readXYH()` *during* `readTransformed()` (`OtosSensor.cpp:268`). If this tick's I2C transaction fails, `raw[6]={0}` decodes to pose (0,0,0) and velocity (0,0), which are passed to `correctEKF()` this tick; the failure is only caught on the *next* call. The Mahalanobis gates are the backstop — fine far from the origin, but near (0,0) a zero-filled read is accepted, and a zero velocity update drags `fusedV` down (the D9 symptom, one tick at a time). Fix: have `readTransformed`/`readVelocityTransformed` return success, and skip fusion on same-tick failure.

### N10 (Medium): `HALT TIME`/`HALT DIST` baselines default to boot epoch — instant trip without `ZERO T`/`ZERO D`

`HaltController::_timerBaselineMs`/`_distBaselineMm` default to 0 (`HaltController.h:87-88`); `evaluate()` builds the baseline from them (`HaltController.cpp:139-140`). `HALT TIME 5000` registered two minutes after boot without a prior `ZERO T` fires on the next tick — an unexpected HARD X (and it wipes all other halt entries when it fires). Either baseline TIME entries at `add()` time, or reject TIME/DIST registration when the baseline was never set. Also: `remove()` deactivates but never frees a slot — `add` always appends at `_entries[_count]`, so 8 cumulative adds fill the table for the session even if all were removed (`HaltController.cpp:17-48`); and one fired condition clears *all* registered conditions (`clearAll()` at `:161`), which is at least worth documenting on the wire.

### N11 (Low-Medium): PURSUE re-gate emits a spurious `EVT cancelled` mid-G

The backtrack re-gate cancels the PURSUE MotionCommand with HARD (`MotionController.cpp:698`), and `MotionCommand::cancel()` emits `EVT cancelled #<corrId>` via the captured sink — the G command's correlation id. A host treating `EVT cancelled` as terminal for that id concludes the G failed, then later receives `EVT done G #<same id>`. Suppress the EVT for internal phase transitions (a `cancelQuiet()` or clearing the sink before cancel, as `_startPreRotate`'s no-sink design already does for PRE_ROTATE).

### N12 (Low-Medium): full `GET` dump exceeds the 255-byte serial TX buffer

The registry is ~50 keys; the `CFG` line builds into a 768-byte buffer (`ConfigRegistry.cpp:165`) and realistically runs 600–800 bytes. CODAL's TX buffer is capped at 255 (`SerialPort.cpp:17`, with a comment saying bursts must fit), and `sendReliable`'s wait can never make room for a line longer than the buffer — it spins 5 ms then hands the whole string to ASYNC, which drops the overflow. Net: bare `GET` over serial is truncated mid-keys. Verify on the bench; if confirmed, chunk the dump into multiple ≤200-byte `CFG` lines.

### N13 (Low): residual dead/vestigial code

`RatioPidController` is constructed, reset, and SET-tunable (`pid.*` keys) but its `update()` never runs in `controlTick` — the sync-gain coupling replaced it. `PID_BYPASS` remains (`MotorController.cpp:12`). `Odometry::update()` is deprecated with no callers. `DriveMode::TIMED` is unreachable (T runs as VELOCITY), so TLM `mode=` can never read `T` — check host parsers don't expect it. Each is small; together they recreate last round's "multiple plausible stories" problem in miniature.

### N14 (Low): queued-path `corrId` truncation to 7 chars

`ParsedCommand::corrId` is 8 bytes (`CommandTypes.h:158`) while the tokenizer, `MotionCommand`, and `TargetState` all carry 16. A host using >7-digit correlation ids (e.g. ms timestamps) gets silently truncated ids on every queued reply and EVT, breaking host-side correlation. Make the sizes uniform (16) or reject long ids loudly.

### N15 (Low): EKF process noise is loop-rate-coupled

`Odometry::predict()` runs every loop iteration and `EKF::predict()` adds full `Q` per call, ignoring `dt_s` (`EKF.cpp:149`). The real iteration period swings ~10–25 ms with I2C load, so effective process noise varies ~2.5× with bus traffic. P2.3.1 from the plan (scale Q by dt or gate predict) was not done. Tuning currently absorbs it; it will resurface as "EKF behaves differently when the color sensor is enabled."

### N16 (Low): invalid `sensor=` stop silently ignored on the queue path

On the direct path, a malformed `sensor=` token replies ERR and cancels; on the queue path (`handleVW` T/D/TURN branches, `MotionCommandHandlers.cpp:784-793` etc.) parse failure just skips the stop — the command runs without its sensor trigger after the host already got `OK`. The TIME net bounds the damage, but a line-following stop that never fires is a behavioral surprise. Validate in the converter (before replying OK) instead.

---

## Scorecard

| View | Score | Movement | Rationale |
| --- | ---: | --- | --- |
| Architecture, modularity, cohesion | 4 / 5 | ↑ from 3 | Layering (types→hal→control→app→robot) is now real; `loopTickOnce` unifies sim and firmware on paper. Docked for the Phase-3 queue wipe undoing that unification in practice (N2). |
| Command-to-motion execution paths | 3 / 5 | = | Stop-condition machinery and TIME nets are solid; the begin*() preemption matrix is inconsistent (N4, N5) and queue overflow is silent (N7). |
| Embedded runtime, timing, concurrency | 4 / 5 | ↑ from 3 | Rollover handling is now uniformly signed; watchdog semantics are clean. Docked for the TLM null-fn/ctx-mismatch (N3). |
| Robotics model, numerics, hardware safety | 3 / 5 | = | EKF + heading fusion + gate recovery are well executed; encoder-reset pose corruption (N1) is the single biggest field risk and is a repeat finding. |
| Interpretability, dead code, change safety | 3 / 5 | ↑ from 2 | Old schedulers deleted, history well-commented. Vestigial PID/flags remain (N13), and the queue-mode flip (N2) makes runtime behavior history-dependent. |

## Suggested fix order

1. N1 (atomic encoder reset) and N2 (one-line queue re-wire) — both are small and remove pose corruption and a whole class of "works in sim, differs on robot."
2. N3 (TLM null/ctx) — crash-grade, three lines.
3. N4 + N5 (uniform cancel-if-active across all begin*) — one pattern, five call sites.
4. N7 (ERR on queue full), N6 (validateConfig additions), N10 (HALT baselines).
5. The rest as cleanup tickets.

## Residual risk

Read-only review; nothing was compiled or simulated. The highest-value regression tests to add, in order: D-then-G pose continuity with fusion off (catches N1), a firmware-config boot test asserting `cmd` still has its queue after Phase 3 (catches N2 forever), `SET tlmPeriod` without STREAM (N3), and `S` injected mid-TURN on the queue path (N4 — this is the P1.1 verify scenario, which does not currently pass as specified).
