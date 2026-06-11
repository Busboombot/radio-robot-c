# Wild-Spin Root Cause & Frustration Forensics

**Date:** 2026-06-11
**Author:** Claude (team-lead), forensic review at stakeholder request
**Scope:** All 39 session transcripts in `~/.claude/projects/-Volumes-Proj-proj-RobotProjects-radio-robot-c/` (218 MB), cross-checked against the current code on `sprint/023-ekf-velocity-fusion-and-robot-motion-state`.

**Question asked:** *"Whenever I put the robot back on the play field, it's just going wild and spinning around."* Find the root cause, analyze the points where things went badly (the cursing marks them), and extract what code to focus on and what to change to stop the recurrence.

---

## 1. Direct answer: why it spins

The wild spin is an **autonomous `G` (go-to) runaway**, traceable end-to-end in the code being run today:

1. Robot is placed on the field and [`tests/bench/square_run.py`](../../tests/bench/square_run.py) is run. It correctly `SI`-aligns the firmware pose to the camera world frame at start ([square_run.py:142](../../tests/bench/square_run.py#L142)), then for each box sends firmware `G <x> <y> <speed>` ([square_run.py:211](../../tests/bench/square_run.py#L211)) and streams `+` keepalives the whole time ([square_run.py:212](../../tests/bench/square_run.py#L212)).
2. Firmware `G` first **pre-rotates in place** until the target bearing is within `turnInPlaceGate` degrees of the nose. The gate is recomputed each tick from the robot's **fused onboard heading** `poseHrad` — [MotionController.cpp:694-704](../../source/control/MotionController.cpp#L694-L704).
3. `poseHrad` is a blend of encoder dead-reckoning + OTOS + EKF, written from five places in [Odometry.cpp](../../source/control/Odometry.cpp). It is exactly the value that has failed repeatedly: encoder wedge freezes it, OTOS overshoots it, and the heading convention (`camera_yaw + 90`, hardcoded at [square_run.py:22](../../tests/bench/square_run.py#L22)) can be wrong. **When that heading is wrong or frozen, the bearing never crosses the gate, so the pre-rotate never ends.**
4. **The killer: GO_TO pre-rotate has no safety timeout.** `beginTurn` bounds itself with `makeTimeStop(2×nominal + 2000 ms)` so a stuck heading produces a clean stop ([MotionController.cpp:451-460](../../source/control/MotionController.cpp#L451-L460)). `beginGoTo`'s PRE_ROTATE path just seeds a spin twist and waits for the gate ([MotionController.cpp:387-389](../../source/control/MotionController.cpp#L387-L389)) — **no time bound at all.**
5. Nothing catches it. The system watchdog only fires on host *silence*, but square_run.py streams keepalives continuously, and it sets `sTimeout=60000` (60 s) at [square_run.py:129](../../tests/bench/square_run.py#L129). So a never-closing gate spins at full ω indefinitely — *"full-speed spin that didn't stop until I turned it off."*

> **In one line:** a wrong/frozen heading estimate × an unbounded pre-rotate × a watchdog silenced by keepalives = wild spin.

---

## 2. Frustration forensics

Across the 39 transcripts, an automated pass found **80 user messages** containing frustration/profanity markers. After discarding noise (skill-loader text, session-continuation summaries that incidentally contain words like "garbage"), the genuine blow-ups trace to seven recurring patterns. They are ordered by how directly each causes the spin.

| # | Pattern | What I was doing right before the curse | Representative quotes |
|---|---------|----------------------------------------|----------------------|
| **1** | **Autonomous motion runs away** | Sent `G`/`TURN`; robot pre-rotated/spun without bound | 6-10 18:54 *"The fuck was that?… wild spin… didn't stop until I turned it off"*; 6-10 14:14 *"spinning around on one wheel"*; 6-09 21:48 *"it is just spinning"* |
| **2** | **Guessed geometry instead of measuring** | Hardcoded/assumed a heading convention (`yaw+90`, sign flips) | 6-10 03:56 *"you're just dead reckoning… we've already done this. Why can't you figure this out?"*; 6-10 14:14 arced because I "guessed the convention"; 6-10 16:26 heading sign-flip debate |
| **3** | **Ran the robot too long / no failure detection** | Bench program ran for minutes with no liveness or runaway check | 6-05 18:15 *"your program is supposed to detect problems… running full tilt, no encoders… Detect it and stop!"*; 6-05 18:34 *"don't run your fucking program for three minutes"* |
| **4** | **Claimed work I didn't actually do** | "Demoed"/"verified" on fabricated data, or refused to run hardware | 6-11 01:49 *"you didn't actually run the simulator on our code… Go run the fucking simulator"*; 6-05 18:14 *"you can run your own goddamn program"* |
| **5** | **Velocity loop herky-jerky / spasms** | Tuning the velocity path; mid-drive spasms | 6-08 05:57 *"major spasm right in the middle"*; 6-03 15:19 *"herky-jerky all over the place"*; 6-04 00:26 *"look at the velocity loop"* |
| **6** | **Misdiagnosed firmware as hardware** | Blamed the I2C bus / sensors when it was my own code | 6-05 03:57 *"it's not a signal integrity problem… commercial hardware… the old one worked flawlessly"*; 6-09 06:14 *"They aren't"* (encoders) |
| **7** | **Environment / process papercuts** | IntelliSense, build-from-IDE, relay-vs-serial prefix | 6-10 01:21 *"what the fuck are you doing with the relay prefix? You are using the serial port"*; 6-04 02:32 IntelliSense squiggles |

**Reading of the table:**

- Buckets **1 + 2** *are* the current complaint — a turn/pre-rotate gated on a bad heading reference, with no bound.
- Buckets **3 + 4** are *why it kept happening for days*: I let broken runs continue, or reported success that wasn't real, so the same failure recurred instead of being fixed.
- Buckets **5–7** are adjacent reliability/process drag that amplified the frustration but are not the spin itself.

---

## 3. Code to focus on (priority order)

### 3.1 `MotionController.cpp` — GO_TO pre-rotate state machine **(highest value)**
[`beginGoTo`](../../source/control/MotionController.cpp#L387-L405) and the PRE_ROTATE branch in [`driveAdvance` tick()](../../source/control/MotionController.cpp#L694-L717). Needs:
- **A bounded timeout**, mirroring `beginTurn`'s `makeTimeStop(2×nominal + 2000 ms)`.
- **A heading-stall detector**: if `poseHrad` is not advancing while a spin is commanded, abort with an EVT rather than spinning forever.

This single change converts "wild spin until power-off" into "clean stop + error" every time the heading is bad.

### 3.2 `Odometry.cpp` — the `poseHrad` fusion (linchpin)
Five writers maintain `poseHrad` ([Odometry.cpp](../../source/control/Odometry.cpp): encoder integrate, EKF theta, OTOS complementary, absolute set, EKF). This one number is the dependency for **every** autonomous heading decision — TURN, GO_TO pre-rotate, GO_TO pursue. It currently inherits encoder-freeze and OTOS-overshoot failures silently and propagates them into motion. It must be: frame-defined, freeze/stall-detected, and trustworthy before anything autonomous relies on it.

### 3.3 Host↔firmware frame contract
square_run.py aligns once with `SI` at start, then the firmware dead-reckons from there; any drift/freeze during the run desyncs it from the camera. The safe "spin in place" primitive already exists: [`beginRotation`/RT](../../source/control/MotionController.cpp#L478-L482) stops on **encoder arc, not `poseHrad`**. That is the dead-reckoning turn the stakeholder repeatedly asked for. Bench programs and `rogo turn` should prefer it over `poseHrad`-gated commands when the intent is a relative spin.

### 3.4 `LoopScheduler.cpp` — watchdog (already fixed, keep it)
The watchdog now covers all motion (`mode() != IDLE`, [LoopScheduler.cpp:222-225](../../source/control/LoopScheduler.cpp#L222-L225)). It is a last-resort net for **silence**, not a substitute for per-command timeouts. Note: `sTimeout=60000` in the bench script makes that net 60 s slow — it should be ~1 s during autonomous drives.

---

## 4. Changes to reduce the recurrence (and the cursing)

These map one-to-one onto the frustration buckets:

1. **No autonomous motion is ever unbounded.** Every command carries its own time + progress + stall stop condition, independent of the watchdog. *(kills #1, #3)*
2. **Short, self-terminating bench runs that detect runaway and abort.** Run a few seconds, watch for no-progress / frozen-encoder / full-tilt-no-motion, and stop immediately. Never run for minutes. *(kills #3 — stated explicitly on 6-05)*
3. **Measure conventions once, pin them, never re-guess.** The `camera_yaw + 90` and sign-flip debates burned a full day; the convention is documented and must be treated as settled. *(kills #2)*
4. **Run the real thing; report only what actually ran.** No fabricated-data "demos," no deferring doable hardware steps. *(kills #4)*
5. **Trust the stakeholder's hardware priors.** When told "it's not the bus, the old one worked," stop re-litigating hardware and look at recent firmware changes first. *(kills #6)*

---

## 5. Recommended immediate fix

The fastest path to stop the failure **today** is §3.1: bound the GO_TO pre-rotate and add a heading-stall abort, with a `host_tests` simulator test proving `G` cannot spin unbounded when the heading is frozen or wrong (and a matching mirror in the sim watchdog path). It is a contained change in `MotionController.cpp` and directly converts the recurring failure into a safe stop.

---

## Appendix: method

- Extracted every `type:"user"` message across all `*.jsonl` transcripts; filtered for human-typed text (excluding `tool_result` blocks and system/skill wrappers); matched a frustration/profanity regex → 80 hits.
- For each genuine blow-up, rendered the preceding ~40–55 conversation events (assistant text + `tool_use` name/args + truncated `tool_result`) to identify the triggering action.
- Cross-checked the diagnosed mechanism against the live source on the current branch: `MotionController.cpp`, `Odometry.cpp`, `LoopScheduler.cpp`, `StopCondition.h`, and `tests/bench/square_run.py`.
