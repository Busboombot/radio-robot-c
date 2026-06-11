# Improvement Plan — Sim-to-Real Recovery

**Date:** 2026-06-11
**Prerequisite reading:** `2026-06-11-sim2real-architecture-review.md` (defects D1–D12, scenarios).
**Audience:** the main (team-lead) agent. Work the phases in order; each phase is independently
testable and leaves the robot better than before. Do not reorder P0 items — they are sequenced so
that each fix is observable through the previous one.

Ground rules for this effort:

- **Never use `SAFE off` again.** P0.1 makes it unnecessary. Treat any future urge to disable the
  watchdog as a signal that a stop condition or keepalive is missing.
- **Every fix lands with (a) a sim test that reproduces the failure with slip + fusion enabled, and
  (b) a 2-minute hardware check.** A fix verified only in the default (slip-free, fusion-off) sim
  configuration is not verified.
- Tune/calibrate values live in `data/robots/tovez.json` → regenerate `DefaultConfig.cpp` via
  `scripts/gen_default_config.py`; do not hand-edit constants into code.

---

## Phase P0 — Stop the bleeding (field safety + heading truth)

### P0.1 Host: send `+` keepalives while waiting; firmware: bound every motion phase

**Why first:** removes the reason `SAFE off` ever gets typed, restoring the safety net before
anything else is field-tested.

1. Keepalive status: as of 2026-06-10, `serial_conn.py` already streams `+` every 150 ms from a
   background daemon for the life of the connection — keep it (it is the right dead-process
   detector). But understand what it changed: the firmware watchdog now never fires while the host
   process lives, so it no longer supervises motion at all. Items 2–4 below are therefore not
   hardening — they are currently the **only** thing standing between a stalled stop condition and
   an unbounded spin. Additionally, make `+` quiet on the wire (suppress the `OK keepalive` reply
   in `handleKeepalive`, or honour a `+q` variant): at 6.7 Hz the acks are pure noise competing
   with TLM for the 250-byte TX buffer, and the host already filters them out on receipt.
2. `MotionController::beginGoTo()` PRE_ROTATE branch — replace the raw BVC seeding with a proper
   MotionCommand so the phase is supervised like every other motion:
   - `_activeCmd.configure(0.0f, omega, &_bvc)` (also fixes D7 — configure() clears any stale command;
     additionally call `_activeCmd.cancel(HARD)` first if `_activeCmd.active()` to emit the stale
     command's cancellation explicitly rather than silently absorbing it),
   - `addStop(makeHeadingStop(bearing_delta, gateRad))` — stop when rotated to within the gate,
   - `addStop(makeTimeStop(2×nominal + 2000))` — same runaway net TURN/RT already have,
   - on completion, transition to PURSUE (start the pursuit MotionCommand exactly as the current
     PRE_ROTATE→PURSUE transition does).
   - Drop the `seedCurrent(0, omega)` jump-start; let the BVC ramp under `yawAccMax` (the instant
     180°/s start is both the "fast spin" signature and a slip generator).
3. `beginGoTo()` PURSUE path — `addStop(makeTimeStop(...))` overall net for G:
   `2 × (distance/speed)·1000 + 4000` ms. G is currently the only motion command with no time bound.
4. `beginTurn()`/`beginVelocity()`/`beginArc()` — same explicit-cancel-if-active rule as (2).

**Verify (sim):** new test — issue G with target at 135° bearing while a VW command is active and
heading frozen (inject via mock): command must end with the TIME net, not spin forever; no stale
EVT labels. **Verify (hardware):** `G` to a behind-the-robot target with the host deliberately
silent: robot must stop on its own within the time net, safety ON the whole time.

### P0.2 Watchdog: distinguish link-loss from motion-supervision (firmware)

`LoopScheduler::run_blocks()` watchdog: with P0.1 every motion phase has its own TIME net, so the
keepalive watchdog's job reduces to link-loss. Two changes:

1. Exempt MotionCommands that carry at least one TIME stop from the keepalive requirement
   (query: `_activeCmd.hasTimeStop()` — add the trivial accessor). S/`_VW`/R (open-ended) remain
   keepalive-bound. This makes `go_to()`/`turn()` safe even from hosts that forget keepalives,
   without removing the net for streaming modes.
2. `SAFE off` must not be a permanent foot-gun: auto-re-arm `safetyEnabled = true` whenever a new
   motion command begins (one-shot disable semantics), and emit `EVT safety re-armed` so the host
   sees it. Update `tests/dev/safe_cmd_bench.py` to the new semantics.

**Verify (sim):** T/D/G/TURN complete with zero keepalives, safety on; S without keepalives still
safety-stops at `sTimeoutMs`.

### P0.3 Fuse OTOS heading into the EKF (kills D1)

1. `EKF.h/.cpp`: add `updateHeading(float theta_meas, float r_theta)` — scalar update, H = [0,0,1,0,0],
   **wrap-safe innovation**: `y = wrapPi(theta_meas − _x[2])`; gate at χ²(1) 3.84 like the velocity
   updates, with the same escape hatch as P0.4. Update `_x`, P rows/cols exactly per the existing
   scalar-update pattern (`updateVelocity` is the template).
2. `Odometry::correctEKF()` — accept `theta_otos_rad` and call `updateHeading` between position and
   velocity updates. `Robot::otosCorrect()` already has `p.h` in hand — pass it through.
3. New config: `ekfROtosTheta` (start ≈ 0.01 rad² ≈ (5.7°)²; tune down if OTOS heading is clean).
4. While here: `EKF::setPose()` must stop zeroing P — set the diagonal to a sane prior
   (e.g. 100 mm², 100 mm², (5°)², and leave velocity variances) so the filter can absorb errors in
   the supplied pose and the gates aren't strangled right after a reset.

**Verify (sim):** enable `sim_set_otos_fusion(true)` + mock slip (`slipTurnExtra` ≈ 0.26-equivalent);
run square + figure-eight; fused heading must track mock-OTOS truth within ~2°, where today it
drifts per-turn. **Verify (hardware):** four TURN 9000s in a row must return the robot to its
starting orientation within a few degrees (today: ~off by ~90° physically).

### P0.4 Mahalanobis gate: add a recovery path (kills D3)

In `EKF::updatePosition` (and the new `updateHeading`): count consecutive rejections; after
N = 10 consecutive, inflate S (e.g. scale R by 10× for that update) or accept unconditionally once,
then reset the streak. Telemeter `ekf_rej` (cumulative reject count) in the TLM frame so divergence
is visible from the host. This converts "permanently lost" into "recovers within ~1 s".

**Verify (sim):** teleport the mock-OTOS pose 200 mm mid-run (fusion on): fused pose must converge
to the new OTOS truth in < 2 s instead of free-running forever.

### P0.5 Apply `rotationalSlip` where rotation is measured (kills D2)

Decision to make explicit in the commit: with P0.3 in place, OTOS heading is the primary heading
truth, and encoder dθ is the *prediction*. Still correct the prediction:

1. `Odometry::predict()`: `dTheta = ((dR − dL) / trackwidthMm) * cfg.rotationalSlip;` — encoders
   over-report body rotation on real surfaces; 0.74 is the measured efficiency. Gate it: clamp the
   configured value to [0.5, 1.0] and treat 0/unset as 1.0 so old configs don't break.
2. `beginRotation()` (RT): divide the target arc by `rotationalSlip` (the wheels must travel
   *farther* than the no-slip arc to achieve the angle).
3. Sim parity: `MockMotor` slip defaults stay 0 (it's ground truth for unit tests), but add a
   "field profile" sim fixture that sets mock slip + fusion on; all G/TURN regression tests run in
   BOTH profiles from now on.
4. Note the mock slip sign convention models encoder *under*-report; physical turn slip makes
   encoders *over*-report rotation. Fix `MockMotor`'s turn-slip term to add scrub (encoder > body
   rotation) so the field profile actually reproduces the field failure direction.

**Verify (hardware):** TURN 9000 lands 90° ± 3° physical (tape-measure/protractor or OTOS readout);
RT 9000 likewise.

---

## Phase P1 — Command pipeline correctness

### P1.1 Keepalives must never mutate an active command (kills D6)

`handleVW` no-stop-params branch: only treat the VW as a keepalive-with-retarget when the active
command **is a plain VW session** (track a `MotionCommand::Origin` enum set at begin time:
VW/TURN/G/T/D/R/RT). For any other origin: reset the watchdog, reply `OK vw busy=<origin>`, and do
NOT `setTarget`. The host keepalive story becomes: `+` for everything; `VW` re-send only retargets
VW sessions. Update `protocol.py` docstrings (`vw()`, `drive()`) which currently *recommend* the
destructive pattern.

**Verify (sim, queue wired — see P1.3):** start TURN, inject `S 0 0` mid-turn → TURN must complete
at the commanded heading.

### P1.2 One reply per command (kills D11)

Converters (`handleS/T/D/G/R/TURN/RT`) currently reply AND `handleVW` replies again. Pick one owner:
the converter keeps the user-facing reply (`OK goto …`), and `pushVW` marks the ParsedCommand
`quiet=true` so `handleVW` skips its `replyOK` for converted commands (direct VW commands still get
`OK vw`). Wire-protocol docs (`docs/protocol-v2.md`) updated accordingly.

**Verify:** `host/tests/test_protocol_v2.py` — count exactly one OK per command on the queue path.

### P1.3 Make the sim run the real dispatch path (kills the biggest sim/real split)

1. `host_tests/sim_api.cpp`: instantiate a `CommandQueue`, call `cmd.setQueue(&q)` and
   `robot.motionController.setQueue(&q)`, and drain it in `sim_tick()` exactly via
   `cmd.dequeueOne(q)` — the same calls `run_blocks()` makes. The direct `begin*()` fallbacks then
   exist only for unit tests that target them explicitly.
2. Medium-term (flag as its own ticket): extract the body of `run_blocks()` into a
   `LoopScheduler::tickOnce(now)` that both the firmware loop and `sim_tick()` call, deleting the
   hand-mirrored copy in sim_api.cpp. The "MUST mirror exactly" comment is the bug report.

**Verify:** `test_vw_converters.py` passes against the queue path; double-OK test from P1.2 runs in
sim and would have caught D11.

### P1.4 Pursuit-law hardening (kills D8)

In the PURSUE per-tick hook:
1. Clamp curvature: `|κ| ≤ κMax = 2 / max(d_remaining, 2·arriveTolMm)` or equivalently clamp ω to
   what `BodyKinematics::saturate` can express without reversing a wheel; pick one and document it.
2. Re-gate: if `|bearing| > 90°` (target behind) for more than ~3 consecutive ticks, drop back to
   PRE_ROTATE (now safe and supervised after P0.1) instead of orbiting.
3. Widen `arriveTolMm` for field use (5 mm is sim fantasy; 20–25 mm is realistic on carpet), and
   make the G POSITION stop radius ≥ the worst-case decel distance at commanded speed so SOFT
   ramp-down lands inside the disc.

**Verify (sim, field profile):** targets at 0°, ±90°, 180°, and 30 mm lateral offset all converge,
no orbit > 1.5 revolutions in the log.

---

## Phase P2 — Sensor and telemetry robustness

### P2.1 OTOS validity gating (kills D9; directly addresses spin-on-placement)

1. `OtosSensor`: read the STATUS register (0x1F on SparkFun OTOS: tilt + optical tracking warn/fatal
   bits) in `otosCorrect()`'s cadence *before* using pose/velocity; on warn/fatal or I2C read
   failure, set `state.inputs.otos.valid = false` and **skip fusion entirely** that tick.
2. Distinguish "I2C returned zeros" from "pose is genuinely (0,0,0)": check the read return path in
   `readXYH` and propagate a bool instead of silently keeping zeroed int16s.
3. While invalid > ~500 ms during active motion, emit a one-shot `EVT otos lost` so the host knows
   pose quality degraded (and the agent stops blaming the controller).
4. Fix the mounting-offset transform (dormant D9 tail): the offset must be applied in the sensor
   frame and rotated by current heading, not subtracted as world constants. Keep no-op behaviour
   for zero offsets.

**Verify (hardware):** lift robot mid-G: motion stops via stop conditions, `EVT otos lost` appears,
no spin on placement; pose recovers after `SI`/camera fix.

### P2.2 Telemetry stream the host can trust (kills D10, D11a)

0. **Host, first and cheapest:** delete `reset_input_buffer()` from `SerialConnection.send()`
   (D11a — it discards buffered TLM/EVT lines on every synchronous command, including the
   completion events other code is waiting for). Replace the read-after-write pattern with a single
   reader thread that demultiplexes incoming lines into (a) a reply queue keyed by corr-id, (b) a
   TLM stream queue, (c) an EVT queue — so synchronous sends and stream consumers stop fighting
   over one input buffer. This one change will likely resolve most "stream keeps dying" and
   "EVT done never arrived" reports regardless of firmware work.
1. Add `seq=<n>` (uint16 wrap) to the TLM frame so the host can measure drop rate; surface it in
   `TLMFrame` parsing.
2. Replace the idle-silence design with a low idle rate: when IDLE > grace, emit at
   `max(tlmPeriodMs, 500)` instead of nothing. The host's "is the link alive" question should be
   answerable from the stream itself. Document it in `protocol-v2.md`.
3. Bind the TLM channel explicitly: `STREAM` captures its reply channel as the stream sink;
   commands on other channels no longer steal the stream (`activeTlmFn` only updates on `STREAM`).
4. Move the `tlmPeriodMs < 20` clamp out of `telemetryEmit` and into the `STREAM`/SET handler
   (no config writes from the telemetry path), and reject periods the 250-byte CODAL TX buffer
   can't sustain at the current field set (back-of-envelope: frame bytes × rate < ~60% of
   115200/10 baud-bytes); reply `OK stream ms=<clamped>` so the host knows what it actually got.

**Verify:** host-side drop-rate report from seq gaps < 2% during a full G run over serial relay;
stream survives an idle→drive→idle cycle without the host reconnecting.

### P2.3 EKF housekeeping

1. Gate `Odometry::predict()` to `controlPeriodMs` like `driveAdvance` (or scale Q by dt) so process
   noise isn't loop-rate-coupled.
2. Telemeter `ekf_rej` and the P-trace (P[0][0]+P[1][1], P[2][2]) at low rate for field debugging —
   divergence becomes a number on a chart instead of a robot in the boards.

---

## Phase P3 — Process changes (for the team-lead agent)

1. **Field-profile CI:** every motion-control test runs twice (exact profile + field profile with
   slip, fusion, deadband, 15 ms command latency). A PR that only passes the exact profile is not
   done. Add the four incident scenarios from the review (§4.1–§4.4) as named regression tests.
2. **Hardware smoke ritual** (5 min, scripted in `tests/bench/`): SAFE query (must be `on`),
   TURN 9000 ×4 → orientation closure, G square → return-to-start error < 50 mm, lift-test →
   `EVT otos lost`, stream drop-rate print. Run before and after every firmware flash; log results
   to `docs/knowledge/field-log.md` with date + git SHA.
3. **`SAFE off` is banned in committed code and notebooks.** The bench script that legitimizes it
   (`tests/dev/safe_cmd_bench.py`) gets rewritten for one-shot semantics (P0.2).
4. **Single source of loop truth:** after P1.3's `tickOnce()` extraction, delete the mirrored loop
   in sim_api.cpp and add a comment-lint (grep in CI) for the words "MUST mirror".
5. When debugging field failures, pull `ekf_rej`, seq-gap rate, and mode/phase from TLM **before**
   touching gains. The last month of transcripts shows gain-tuning attempts against what were
   actually estimator/watchdog faults.

---

## Quick reference: defect → fix map

| Defect | Fix |
|---|---|
| D1 heading never fused | P0.3 |
| D2 rotationalSlip unused | P0.5 |
| D3 gate divergence trap | P0.4 |
| D4 watchdog vs. host keepalives | P0.1.1, P0.2 |
| D5 PRE_ROTATE unsupervised spin | P0.1.2 |
| D6 keepalive stomps commands | P1.1 |
| D7 stale MotionCommand race | P0.1.2 / P0.1.4 |
| D8 unbounded pursuit curvature | P1.4 |
| D9 OTOS validity / offset transform | P2.1 |
| D10 telemetry silence/drops/channel | P2.2 |
| D11 double OK | P1.2 |
| D12 misc (Q-vs-rate, clamp-in-emit, …) | P2.3, P2.2.4 |

Estimated order-of-magnitude effort: P0 ≈ 2–3 focused sessions, P1 ≈ 2, P2 ≈ 2, P3 ongoing.
P0.1 + P0.3 alone should visibly transform field behaviour.
