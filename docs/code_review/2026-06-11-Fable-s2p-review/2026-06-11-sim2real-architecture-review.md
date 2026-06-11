# Sim-to-Real Architecture Review — Why the Robot Works in Simulation and Fails on the Playfield

**Date:** 2026-06-11
**Scope:** G (go-to) and TURN command paths, EKF/odometry fusion, safety watchdog, telemetry streaming.
**Companion document:** `2026-06-11-improvement-plan.md` (prioritized, actionable fixes).

---

## 1. Executive summary

The robot's field failures are not random hardware flakiness. They are the predictable output of four
interacting design defects, each of which is invisible in the simulator:

1. **Heading is open-loop.** The EKF never observes heading. OTOS heading is read every 100 ms and
   stored in `HardwareState.otosH`, but it is never fused (`EKF::updatePosition` is x/y only;
   `EKF::updateVelocity` is v/ω only). Heading is pure encoder integration — and the calibrated
   rotational slip factor (`rotationalSlip = 0.74`) is **defined in config but never used anywhere in
   the firmware**. Every in-place rotation physically under-rotates ~26% relative to what odometry
   believes. Heading error accumulates monotonically, and every G command transforms its target into
   the world frame through that wrong heading. That is the "gets turned around and drives into the
   boards" failure, mechanically.

2. **The EKF's outlier gate turns heading drift into permanent blindness.** Once heading drift makes
   dead-reckoned position diverge from OTOS by more than ~2.4σ (Mahalanobis 5.99 with
   `r_otos_xy = 50`), `updatePosition` rejects every OTOS measurement. The filter then free-runs on
   encoders alone with no recovery path. The robot is confidently wrong, forever.

3. **The safety watchdog conflates "link alive" with "motion supervised", and the host doesn't hold
   up its end.** The system watchdog (LoopScheduler) kills *any* motion — including self-terminating
   G/TURN/T/D commands — after `sTimeoutMs = 500 ms` of inbound-command silence. But the host's
   blocking helpers (`go_to()` + `wait_for_evt_done()`, `turn()` + `wait_for_evt_done()`) send
   **nothing** while waiting. So on real hardware every G or TURN longer than 500 ms was killed with
   `EVT safety_stop`. The agent's first workaround was `SAFE off`; its second (2026-06-10) was a
   background daemon that streams `+` keepalives for the life of the connection. Both workarounds
   have the same side effect: the watchdog no longer supervises motion, and it was the **only**
   bound on the G pre-rotate phase (see §4.2). The fast spins are now reachable even with safety
   nominally on.

4. **The simulator exercises a different system than the one that runs on the field.**
   - `sim_api.cpp` never wires a `CommandQueue`, so in sim, S/T/D/G/TURN dispatch through the
     *direct* `begin*()` path; on hardware they go converter → queue → `handleVW` (different code,
     different replies, different timing).
   - OTOS/EKF fusion is **off by default** in sim (`fuseOtos = false`): the sim pose is encoder-only
     dead reckoning over a slip-free `MockMotor` (`_slipStraight = 0`, `_slipTurnExtra = 0`) — so the
     exact mechanism that destroys heading on carpet does not exist in sim.
   - `sim_api.cpp` hand-mirrors the LoopScheduler loop ("MUST mirror LoopScheduler.cpp exactly") —
     a divergence generator by construction.
   - In sim, every `sim_command()` resets the watchdog, and tests/notebooks poll constantly — so the
     watchdog-kills-G failure can't reproduce there either.

Secondary problems compound these: keepalive verbs (S/VW) silently overwrite the targets of active
TURN/G commands; every converter command emits **two** OK replies on hardware (one from the
converter, one from `handleVW`, same corr-id); telemetry goes silent when idle by design and uses a
drop-on-full sender into a 250-byte CODAL TX buffer; and OTOS data is fused without ever checking
the chip's status register, so lifting/placing the robot feeds garbage into the filter.

---

## 2. System architecture (as built)

### 2.1 Firmware control pipeline

```
serial/radio line
      │  runCommsIn() — every loop iteration; resets watchdog on ANY line
      ▼
CommandProcessor::process ──► CommandQueue (push_back)
      ▼  dequeueOne() — ONE command per ~10 ms loop iteration
converter handlers (S,T,D,G,R,TURN,RT)
      │  build VW ParsedCommand w/ "key=value" stop params, push_front
      │  ── and emit "OK <verb> ..." (#corrId)          ◄── reply #1
      ▼  next dequeueOne()
handleVW ──► begin{Stream,Timed,Distance,GoTo,Turn,Rotation,Arc,Velocity}
      │  ── and emit "OK vw ..." (#corrId)              ◄── reply #2 (duplicate)
      ▼
MotionCommand (_activeCmd, single instance) + BodyVelocityController (BVC)
      ▼  driveAdvance() @ controlPeriodMs=10ms
MotorController (per-wheel PI + FF) ──► PWM
```

Pose pipeline, per loop iteration:

```
controlCollectSplitPhase: read encoders (I2C), outlier-clamp, velocity PI
Odometry::predict:        dθ=(dR−dL)/track; EKF.predict; pose ← EKF state   (every iteration)
Robot::otosCorrect:       every lagOtosMs=100ms — read OTOS pose+vel (I2C),
                          EKF.updatePosition(x,y)   [Mahalanobis-gated]
                          EKF.updateVelocity(v,ω)×2 [OTOS + encoder, χ²-gated]
                          ── OTOS heading otosH: stored, NEVER fused
```

### 2.2 The two dispatch paths (sim vs. hardware)

| | Hardware (`run_blocks`) | Sim (`sim_api.cpp`) |
|---|---|---|
| Queue | wired (`setQueue`) → converter→`handleVW` path | **never wired** → direct `begin*()` path |
| OK replies per command | **2** (converter + handleVW) | 1 |
| OTOS fusion | always on (100 ms cadence) | **off by default** (`fuseOtos=false`) |
| Wheel slip | real (≈26% rotational loss, surface-dependent) | `MockMotor` slip defaults **0** |
| Encoder reads | I2C, outlier-clamped, retried, can wedge | exact float integration |
| Watchdog feeding | only when host actually sends | every `sim_command()` call |
| Loop body | `LoopScheduler::run_blocks()` | hand-copied mirror in `sim_tick()` |
| Motor deadband / transients | real (≈35 PWM stiction) | none |

Anything validated only in sim validates the left column of almost nothing.

---

## 3. Defect inventory (with code references)

### D1. OTOS heading never fused — heading is open-loop
- `EKF.cpp`: `updatePosition()` observes x,y only; `updateVelocity()` observes v,ω only. No
  `updateHeading()` exists. Cross-block covariance P[0..2][3..4] is explicitly held at zero, so the
  ω observation cannot influence θ either.
- `Robot::otosCorrect()` reads `p.h` into `state.inputs.otosH` and drops it.
- Heading correction via position cross-covariance (P[0][2], P[1][2]) only builds while driving
  *straight* (F[0][2] = −dCenter·sinθ ∝ dCenter); during in-place turns dCenter≈0, so precisely when
  heading error is created, the filter has no way to fix it.

### D2. `rotationalSlip` (0.74) calibrated but unused
- `DefaultConfig.cpp:67` sets it; `ConfigRegistry.cpp:98` exposes it as `rotSlip`; **no other
  reference in `source/`**. `Odometry::predict()` uses raw `(dR−dL)/trackwidthMm`.
- Consequence: TURN's HEADING stop fires when *encoders* say the heading delta is reached. The robot
  physically rotates ~74% of that. A commanded 90° turn yields ~67° physical. The same error
  corrupts `poseHrad` for every subsequent G world-frame transform. (`turnScale`/`distScale` are
  likewise registered but dead.)
- `beginRotation` (RT) computes its encoder-arc target with no slip term either.

### D3. Mahalanobis gate with no recovery path
- `EKF::updatePosition`: `d2 > 5.99 → ++_rejected; return;`. With `r_otos_xy=50` and small P
  (steady state, or zeroed by `setPose`), innovations ≳ 17 mm are rejected. After divergence,
  *every* OTOS fix is rejected; `_rejected` is counted but not telemetered, alarmed, or acted on.
- `EKF::setPose()` zeroes P entirely (false perfect certainty), making post-reset re-acquisition
  slower and the gate tighter.

### D4. Watchdog design forces the host to choose between two failure modes
- `LoopScheduler::run_blocks()` watchdog: fires `EVT safety_stop` + `X` when `sTimeoutMs` (500 ms)
  passes without **any inbound command**, whenever mode ≠ IDLE or a MotionCommand is active.
  Self-terminating commands are deliberately not exempt (comment in code).
- Host `protocol.py`: `go_to()`/`turn()` then `wait_for_evt_done()` — a read-only loop that sends
  nothing itself. Historically nothing fed the watchdog during waits, so every G/TURN > 500 ms →
  safety_stop. Observed agent behaviour: `SAFE off` (and `tests/dev/safe_cmd_bench.py`
  institutionalizes it). With safety off, defects D5/D6 have no backstop.
- **Timeline correction (important):** on 2026-06-10 (commit `ebf80b2`, the most recent change to
  `serial_conn.py`) a background daemon was added that streams `+` keepalives every 150 ms for the
  whole lifetime of the connection. This stops the spurious safety_stops — but it also means the
  watchdog now **never fires while the host process is alive**, regardless of whether the host
  logic is sane. The watchdog has been demoted from "motion supervisor" to "dead-process detector".
  Consequence: the unsupervised PRE_ROTATE spin (D5) is now unbounded **even with `SAFE on`**. The
  three eras line up with the field reports: (1) safety on, no keepalives → G/TURN killed at
  500 ms; (2) `SAFE off` era → spins with no backstop; (3) keepalive-daemon era (current) → spins
  with no backstop *and* safety nominally on. Each `+` also elicits an `OK keepalive` reply
  (~6.7 Hz both ways) competing with telemetry for the 250-byte TX buffer.

### D5. G pre-rotate phase is unsupervised and starts at full spin rate
- `beginGoTo()` (bearing > `turnInPlaceGate`=35°): seeds the BVC **directly** —
  `_bvc.seedCurrent(0, ω); _bvc.setTarget(0, ω)` — with ω = 2·(speed/dirGain)/track
  (≈ 3.2 rad/s ≈ **180°/s at speed 200**, no ramp: `seedCurrent` bypasses the profiler).
- No MotionCommand is created for this phase ⇒ **no HEADING stop, no TIME stop, no stop conditions
  at all**. Exit requires the *fused-pose bearing* to fall under the gate in `driveAdvance()`.
- If heading isn't advancing correctly (slip, encoder wedge/I2C stall, OTOS invalid) the spin is
  unbounded. The watchdog was the only net; `SAFE off` removes it. This is the "fast spin when you
  put the robot down" with high confidence — and the TURN/RT commands got explicit time-bound nets
  in code review precisely because this hazard was recognized, while PRE_ROTATE was missed.

### D6. Keepalive verbs destroy active commands
- `handleVW`, no-stop-params branch: `if hasActiveCommand() → activeCmd().setTarget(v, ω)`.
- Any plain `S l r` or `VW v ω` keepalive (which the host's `stream_drive()` emits, and which the
  docstrings *recommend* as "keepalive") arriving while a TURN or G MotionCommand is active
  **overwrites the command's (v, ω) target**:
  - TURN: ω gets stomped (e.g. to 0) → heading stop never fires → 2×nominal+2 s TIME net fires →
    firmware emits `EVT done TURN` **as if it succeeded**, at the wrong heading. The host then
    issues the next G from a false heading. Silent navigation corruption.
  - G PURSUE: stomped for one tick (pursuit hook re-sets next tick) — a jolt, recoverable.

### D7. `beginGoTo` does not cancel/own the active MotionCommand in the PRE_ROTATE path
- The PURSUE branch calls `_activeCmd.configure(...)` (which implicitly resets a previous command);
  the PRE_ROTATE branch does **not** touch `_activeCmd`. If any MotionCommand is still active (VW
  keepalive session, a prior G/TURN not yet completed), `driveAdvance()`'s top branch keeps ticking
  the *stale* command — with the BVC now seeded to the pre-rotate spin — and the stale command's
  stop conditions (wrong baselines, wrong EVT label) decide when the robot stops, emitting the wrong
  completion event. Race condition with chaotic field symptoms.

### D8. Pursuit law has unbounded curvature and no mid-flight re-gating
- `driveAdvance` PURSUE hook: κ = 2·dy/d². As the robot passes near/abeam the target (small d, dy ≠ 0)
  κ → large; ω = v·κ saturates the wheels into a tight orbit. If a fused-pose correction lands
  mid-pursuit (or the target was computed from a stale pose), the target can end up *behind* the
  robot; the bearing gate is only applied at `beginGoTo` time and in PRE_ROTATE, never re-checked
  during PURSUE. The POSITION stop (`arriveTolMm=5` mm!) may simply never be satisfiable on carpet,
  leaving the orbit running until watchdog — or until the boards, with `SAFE off`.
- G has **no overall TIME safety net** (T/D/TURN/RT all have one; G does not — neither phase).

### D9. OTOS fused without validity checking
- `OtosSensor::readTransformed/readVelocityTransformed` never read the chip's STATUS register
  (tilt warning / optical-tracking-invalid flags exist on the SparkFun OTOS). A lifted or
  just-placed robot, dust, or a too-tall gap feeds zeros/garbage straight into the EKF (the
  position gate may catch it; the *velocity* updates at v=0/ω=0 are well inside the χ² gate and
  actively drag fused velocity to zero — fighting the controller).
- I2C read failure leaves the int16 buffers at 0 → reported pose (−odomOffX, −odomOffY, 0) with no
  error signal.
- The mounting-offset transform in `readTransformed` subtracts `odomOffX/Y` as world-frame
  constants; a real lever-arm correction is heading-dependent. Dormant (offsets are 0 in
  `tovez.json`) but wrong — will bite whoever first sets a non-zero offset.

### D10. Telemetry stream design vs. agent expectations
- `tlmPeriodMs = 0` by default (stream off until `STREAM <ms>`), and `telemetryEmit()` goes silent
  whenever the robot has been IDLE > 400 ms **by design**. An agent watching the stream sees it
  "die" at every stop — matching the reported "trouble maintaining the stream".
- TLM frames use `send()` = async drop-on-full into CODAL's ~250-byte TX buffer; a 100+ byte frame
  at 50 Hz (clamp floor 20 ms) over the relay will drop frames under any backpressure — with no
  sequence number, so the host can't even detect loss.
- `activeTlmFn` retargets to whichever channel sent the **last** command: a single radio command
  silently steals the serial telemetry stream.
- Double-OK (D11) pollutes the same stream the host parses for TLM/EVT.
- `telemetryEmit` mutates `config.tlmPeriodMs` (clamp) — a config write hidden in the TLM path.

### D11a. `SerialConnection.send()` destroys the telemetry stream it is supposed to share
- `serial_conn.py::send()` calls `self._ser.reset_input_buffer()` before every write: **all
  buffered-but-unread input is discarded** — in-flight TLM frames, async `EVT done`/`EVT
  safety_stop` lines, everything. Any periodic host activity that uses `send()` (SNAP, SET, GET,
  status polls from the CLI/MCP layer) randomly punches holes in the stream and can eat the very
  completion event a `wait_for_evt_done()` elsewhere is blocking on. This is a first-order cause of
  "the agent can't keep the telemetry stream alive" and of motions that "never complete" on the
  host side while the firmware dutifully emitted the EVT.

### D11. Duplicate OK replies on the hardware path
- Converter handler emits `OK goto x=.. #id`; then `handleVW` emits `OK vw x=.. #id` for the same
  command (sim, with no queue, emits only one). Any host logic that correlates by `#id` or counts
  replies sees phantom responses — and this asymmetry exists *only* on real hardware.

### D12. Assorted (flagged for later)
- `Odometry::predict()` runs every loop iteration with no period gate; EKF Q is added per *call*,
  not per second — process noise tuning is implicitly coupled to loop frequency.
- `dequeueOne()` dispatches one command per 10 ms tick; a burst of N commands takes N ticks, adding
  hidden latency between a converter push and its `handleVW` execution (target computed from the
  pose at *handler* time, not arrival time).
- `MotionController::emitEvt`/`MotionCommand::emitEvt` truncate at 48 bytes.
- `Robot::distanceDrive` zeroes `encLMm/R` *after* `beginDistance` captured baselines — fragile
  ordering contract, already bit once (documented in comments).
- Watchdog only arms after the first command **and** requires `activeFn != nullptr`; a robot that
  was driving when the link process died keeps its last BVC target until the watchdog fires — but
  if the host process died *between* boot and first command... not reachable; OK. Keep an eye on
  the `_watchdogMs = now` self-reset (it re-arms rather than latching a stopped state).

---

## 4. Scenario walkthroughs

### 4.1 Scenario: `G 400 300 200` on the real playfield (safety ON)

1. Host sends `G 400 300 200`; `wait_for_evt_done("G", ...)` starts reading.
2. Converter replies `OK goto ...`, pushes VW; next tick `handleVW` replies `OK vw ...` (duplicate)
   and calls `beginGoTo`.
3. Bearing to (400,300) = 36.9° > 35° gate → PRE_ROTATE: BVC seeded to ≈180°/s spin instantly.
4. Robot spins. Encoders over-report rotation by ~1/0.74; fused bearing crosses the gate at a
   physical heading ~10° short. PURSUE starts with a built-in heading error.
5. *(Pre-2026-06-10 host)* t = 500 ms: watchdog sees no inbound commands since the G →
   `EVT safety_stop` + `X`. Robot freezes mid-arc. Host gets "safety_stop". Agent concludes "the
   safety stop is interfering", sends `SAFE off`, repeats.
6. *(Current host, keepalive daemon running)* The watchdog never fires at all. Either way the run
   continues — with a ~10–15° heading lie, a target frozen in a wrong world frame, OTOS fixes being
   Mahalanobis-rejected as the lie grows, κ=2dy/d² tightening the arc as it misses the 5 mm arrival
   disc — and nothing left to stop it before the boards.

### 4.2 Scenario: fast spin on placement (safety OFF)

1. A `G` to a lateral/behind target enters PRE_ROTATE (no stop conditions, no time net) — or a
   stale MotionCommand is still active when G arrives (D7).
2. The robot is lifted (operator repositioning it / it just hit the boards). Wheels unloaded; when
   placed, OTOS optical tracking re-acquires with garbage/zero pose; encoder heading and OTOS
   disagree wildly; position fixes rejected (D3); fused bearing never settles below the gate.
3. PRE_ROTATE's exit condition never fires. ω stays at ≈180°/s. With `SAFE off` — or with the
   current always-on `+` keepalive daemon feeding the watchdog — nothing fires. The robot spins
   until someone grabs it or sends X.
4. Variant: TURN stalled the same way is *eventually* caught by its 2×nominal+2 s TIME net — which
   is why the spins are reported for G but "sometimes" resolve themselves (TURN) after a couple of
   seconds.

### 4.3 Scenario: `TURN 9000` looks perfect in sim, under-rotates ~23% on carpet

1. Sim: `MockMotor` slip = 0, queue unwired → direct `beginTurn`. Encoder-integrated heading IS
   ground truth; HEADING stop fires at 90.0° ± eps. Test passes, plots look beautiful.
2. Field: wheels scrub during the spin; encoders say 90° when the chassis is at ~67–70°.
   `rotationalSlip` (0.74) — measured during calibration precisely to fix this — is never applied.
3. Firmware reports `EVT done TURN`, `poseHrad` = 90°. Physical heading ≈ 68°.
4. Every subsequent G transforms (tx,ty) through a heading that is ~22° wrong; targets land
   sideways; the robot "doesn't seem to understand where it's going."
5. OTOS *knows* the true heading the whole time, in `otosH`, unfused.

### 4.4 Scenario: keepalive kills the TURN (safety ON, agent "doing it right")

1. Agent learns G/TURN die at 500 ms, finds `stream_drive()`/`vw()` ("keepalive — resets watchdog")
   and streams `VW`/`S` while waiting for `EVT done TURN`.
2. Each keepalive hits `handleVW`'s no-stop-params branch with a TURN MotionCommand active →
   `setTarget(v_keepalive, ω_keepalive)` — the TURN's ω is overwritten (D6).
3. The spin stops/changes; the HEADING stop can't fire; the TIME net fires; firmware emits
   `EVT done TURN` **with the original corr-id**, at whatever heading it happened to be.
4. Host believes the turn succeeded. Compounded with 4.3, navigation state is now fiction.

### 4.5 Scenario: "the telemetry stream keeps dying"

1. Host enables `STREAM 40`. Robot drives; frames flow.
2. Motion completes → 400 ms grace → `telemetryEmit` goes silent **by design**. Agent's monitoring
   reads this as a stream failure and starts debugging the serial layer.
3. Robot drives again, frames resume; under load CODAL's 250-byte TX buffer fills and `send()`
   silently drops frames mid-line (host sees truncated lines, no seq numbers to detect gaps).
4. Any command sent over the radio retargets `activeTlmFn`; the serial listener's stream vanishes
   entirely until the next serial command.
5. Double-OKs interleave with TLM lines. The host parser's robustness determines survival.

---

## 5. Root cause, in one paragraph

The system dead-reckons heading from encoders that demonstrably slip ~26% in rotation, never
consults the absolute heading sensor it reads every 100 ms, and gates away the position sensor as
soon as the resulting drift exceeds ~17 mm — so the pose estimate is structurally divergent on real
surfaces. The watchdog design then punishes exactly the commands that need supervision the most,
training the operator (human or agent) to disable the one mechanism that contains the unsupervised
pre-rotate spin. The simulator cannot catch any of this because it runs a different dispatch path,
zero slip, and no sensor fusion by default. Each fix is individually small; the priority order in
the improvement plan matters more than any single change.
