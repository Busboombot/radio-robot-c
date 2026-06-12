# Bench 032 findings — root-cause diagnosis (code analysis)

Analysis of the three bench-032 findings, traced through the firmware source.
Companion to `.clasi/issues/fr-bench-dbg-otos-no-reply.md`,
`fr-bench-twist-fusedv-zero.md`, `fr-bench-right-encoder-wedge.md`.

---

## STAKEHOLDER DIRECTIVE — bench testing uses the SERIAL PORT, not the radio

**Do not run bench tests over the radio relay. The robot is on the stand,
physically next to you, with a USB cable available — the whole point of the
bench is that you are hooked up to the serial port. Use the serial port.**

This is not a style preference; it is the root of most of this session's
confusion:

- The DBG subsystem deliberately replies on serial (`ForceReply::SERIAL`).
  Run the bench over USB serial and every DBG command replies exactly as
  designed — finding 2's "no reply" symptom does not exist on the correct
  transport.
- The radio relay adds an unreliable hop (the data plane drops async output,
  and the `DBG OTOS BENCH 1` enable provably never took effect on the robot)
  on top of a transport the DBG commands were never meant to answer on.
- Driving the bench through the relay means you cannot distinguish "firmware
  bug" from "relay ate it." Over USB serial that ambiguity disappears.

Rewrite the bench harness (`tests/bench/bench_validation_032.py`) to open the
robot's USB serial port directly. Reserve relay/radio testing for what it
actually validates: the radio link itself.

---

## Finding 2 — `DBG OTOS BENCH` / `DBG OTOS` silent on hardware

**Root cause (high confidence): `ForceReply::SERIAL` routes every matched DBG
reply to the robot's local USB serial; the bench harness listens over the radio
relay and can never see them. The reply path is not "broken" — it is pointed at
a port nobody is reading.**

Chain:

1. Every `DebugCommandable` descriptor is registered with `ForceReply::SERIAL`
   (`DebugCommandable.cpp:694-704`; stated as a design rule in
   `DebugCommandable.h:23-24` — "debug output always goes to the serial port
   regardless of which channel the command arrived on").
2. On hardware, `main.cpp:207/216` wires `cmd.setSerialReply(serialReply,
   &comm.serial())`. In `CommandProcessor::dispatchTable`
   (`CommandProcessor.cpp:107-110`), any matched descriptor with
   `ForceReply::SERIAL` swaps the reply fn to that serial sink — for the OK/ERR
   reply *and* for parse-error (`badarg`) and queue-full (`full`) replies.
3. The 032 harness (`tests/bench/bench_validation_032.py`) talks through the
   relay's radio data plane. Replies sent to the robot's own USB serial never
   reach it.
4. The one reply that *was* observed — `DBG` alone → `ERR unknown` — is emitted
   at `CommandProcessor.cpp:96-100`, *before* the ForceReply override, on the
   originating (radio) channel. That asymmetry exactly reproduces the observed
   signature: unmatched lines reply, matched DBG lines are silent.
5. Sim is green because the host harness never calls `setSerialReply`;
   `_serialFn == nullptr` skips the override (`CommandProcessor.cpp:107`) and
   replies return on the test channel. Textbook sim-green/hardware-dark.

**Corollary:** if the line reaches the robot, the handler *executes* — only the
reply is invisible. So `DBG OTOS BENCH 1` may silently toggle bench mode. (But
see the cross-check below: the 032 telemetry indicates bench mode was NOT
active during the drives, so the enable line most likely never reached the
robot's dispatcher — relay-side drop is the remaining suspect. The serial probe
below discriminates.)

**Verification (do first):** connect USB serial directly to the robot, keep the
relay link open, send `DBG OTOS BENCH 1` over radio. Expect `OK dbg otos
bench=1` on USB and silence on radio. That confirms both the routing diagnosis
and whether the command arrives at all.

**Fix:** per the stakeholder directive above, the correct fix is to run the
bench harness on the robot's USB serial port, where these replies already go.
The firmware is behaving as designed; the harness used the wrong transport.
Do NOT change `ForceReply` to chase radio replies for bench work. (If remote
radio access to these commands ever becomes a real requirement, that's a
separate decision — and note `handleDbgOtos` emits a ~150-char pose line
(`pose_buf[200]`, `DebugCommandable.cpp:520-527`) that may exceed radio
payload limits.)

---

## Finding 3 — `twist=` reads 0,0 while driving

**Root cause (high confidence): the EKF velocity states are only ever written
by `updateVelocity()`, and `updateVelocity()` is only reachable through
`Robot::otosCorrect()` — downstream of the OTOS validity gates. On the bench
stand the real OTOS is lifted/invalid, `otosCorrect()` early-returns every
tick, and the encoder-derived velocity is never fused. v/omega stay at their
init value of 0 forever.**

Chain:

1. `twist=` emits `state.inputs.fusedV/fusedOmega` (`Robot.cpp:509-516`),
   written from `_ekf.v()/_ekf.omega()` in `Odometry::predict()`
   (`Odometry.cpp:73-74`).
2. The EKF predict step is a random walk for the velocity block — it inflates
   covariance but never moves the mean (`EKF.h:13-14`, `EKF.cpp:204-208`). The
   only writer of `_x[3]/_x[4]` is `updateVelocity()`.
3. `updateVelocity()` is called solely from `Odometry::correctEKF()`
   (`Odometry.cpp:203-206`), called solely from `Robot::otosCorrect()`
   (`Robot.cpp:285-288`) — *after* the gates: `is_initialized` (197), STATUS
   byte / `lastReadOk` (218, the D9 lifted-robot gate), same-tick
   `readTransformed` failure (255).
4. On the stand the lifted OTOS reports tracking-invalid status (that is the
   D9 gate's design case) — `otosCorrect` returns at line 236 on every 100 ms
   tick. No update is ever *attempted*, which also explains `ekf_rej=0`:
   rejections are counted inside the update functions (`EKF.cpp:426,463`), and
   none ran.
5. Sim is green because the sim/bench OTOS is always valid, so the velocity
   updates always run.

**The architectural bug:** encoder-velocity fusion (`enc_v`/`enc_omega`,
computed in `predict()` independent of OTOS) is needlessly nested inside the
OTOS-gated path. Encoder velocity is available every tick regardless of OTOS
health.

**Fix:** fuse encoder velocity unconditionally (e.g. its own
`_ekf.updateVelocity(enc_v, enc_omega, _rEncV, _rEncV)` call in the predict
phase or an ungated step in the OTOS block), keeping OTOS pose/heading/velocity
fusion behind the validity gates. Caveat from Finding 1: gate the *enc omega*
observation on both encoders being healthy, or a wedged wheel injects phantom
omega into the fused state (see below).

**Cross-check that ties findings 2+3 together:** if `DBG OTOS BENCH 1` had
actually executed during the run, the bench sensor passes every gate
(`BenchOtosSensor` always-valid by design), fusion would have run every 100 ms,
and twist/ekf_rej could not both have stayed 0 while pose ran to a 131°
phantom heading. So the run provably executed with bench mode OFF — i.e. the
enable command did not take effect, not merely its reply lost. After fixing
the reply routing, re-verify the enable actually lands (relay drop is the
open suspect).

---

## Finding 1 — right encoder wedge corrupting odometry (firmware-hardening angle)

The hardware fault itself is confirmed and out of scope here, but the firmware
currently has detection without defense:

- Detection exists: `MotorController` wedge detector (015-003) —
  `_stuckCountL/R`, `kWedgeThreshold=10`, `EVT enc_wedged` emission
  (`MotorController.h:183-204`).
- Nothing consumes it: `Odometry::predict()` integrates `dL/dR`
  unconditionally (`Odometry.cpp:40-55`). A wedged right wheel turns the
  missing counts into `dTheta = (dR-dL)/track` — the phantom heading swing —
  and EKF predict propagates it into fused pose with no opposing observation
  (OTOS was gated out, Finding 3).

**Hardening options for the odometry side:**
- Expose per-wheel wedge state from `MotorController` (e.g.
  `bool wheelWedged(L/R)` from stuck counters/latches).
- While a wheel is flagged wedged: stop integrating the differential into
  `dTheta` (hold heading; optionally estimate `dCenter` from the healthy wheel
  alone), and mark pose degraded so the host knows the estimate is coasting.
- When encoder-velocity fusion is un-gated (Finding 3 fix), suppress the
  `enc_omega` observation while wedged for the same reason.

Acceptance for the firmware part stays as the issue states: no phantom heading
swing on a single wedged encoder, characterized with `enc_selftest`
before/after.

---

## Suggested fix order

1. **Switch the bench harness to the robot's USB serial port** (see the
   stakeholder directive at the top). This makes the DBG replies visible as
   designed and removes the relay as a confound. Finding 2 then reduces to
   verifying the commands behave on serial; no `ForceReply` change is needed
   for bench work. (A `ForceReply` change is only worth considering if remote
   radio access to these commands is ever a real requirement — do not do it
   as part of this fix.)
2. Re-run bench 032 over USB serial with `DBG OTOS BENCH 1` confirmed via its
   `OK dbg otos bench=1` reply — this alone should make `twist=` nonzero
   (bench passes the gates), separating the transport problem from the fusion
   gating bug.
3. Finding 3 un-gating of encoder-velocity fusion — correct for real-floor
   operation whenever OTOS drops out; sim test: OTOS invalid + wheels moving →
   twist tracks encoder velocity.
4. Finding 1 odometry wedge defense, after the hardware investigation decides
   whether the wedge is fixable at the bus/electrical level.
