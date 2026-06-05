---
status: in-progress
sprint: '014'
tickets:
- 014-001
- 014-002
- 014-003
- 014-004
- 014-005
- 014-006
- 014-007
- 014-008
- 014-009
---

# Plan: Single Cooperative Main Loop (abandon fibers)

## Context

The firmware currently runs **two CODAL fibers** (`source/main.cpp:106`):

- A **control fiber** (`controlFiberFn`) running `robot.controlTick()` every `controlPeriodMs`
  (default 10 ms): encoder I2C read → per-wheel PID → motor PWM → odometry predict →
  OTOS correct (10 Hz) → S/T/D/G drive-mode machines → enqueue completion events.
- A **comms+telemetry fiber** (the `main()` while-loop, 5 ms): drain serial/radio,
  dispatch commands, drain the EVT ring buffer, emit TLM.

The two fibers forced two pieces of complexity that buy us nothing in a single-threaded
model: a lock-free EVT ring buffer to cross the fiber boundary, and **busy-wait I2C**
(`Motor::readEncoderRaw`/`readSpeedRaw`) chosen specifically so the CODAL scheduler could
not dispatch the comms fiber to issue a competing I2C write mid-transaction.

**The new model:** abandon fibers entirely. One cooperative main loop with a **priority
task table**. A hard **control task** (encoder read → PID → PWM write) is the metronome and
always runs first; every other activity is a lower-priority task that runs only in the time
left over before the next control deadline. State is consolidated into **three authoritative
data structures** that the tasks read and write, replacing the state currently scattered
inside the subsystem classes.

Intended outcome: a deterministic, easy-to-reason-about scheduler where the velocity loop
runs at a predictable cadence, sensor reads update at their own configurable rates, and the
data flow (inputs → control → outputs) is explicit in three structs rather than hidden in
private members across five classes.

---

## The three authoritative data structures

New header `source/control/RobotState.h`. These structs **are** the state — the subsystems
are refactored to read from and write to them, not to keep parallel private copies. Types
match the existing code (mm as float internally / int32 mm + centidegrees at the telemetry
boundary, uint16 sensor channels, uint32 ms timestamps).

```cpp
// One async value-set's freshness bookkeeping.
// lagMs = configured update period. 0 = refresh every loop iteration (synchronous).
struct ValueSet { uint32_t lagMs; uint32_t lastUpdMs; bool valid; };

// ---- 1. OUTPUT: what the loop drives onto the hardware ----
struct MotorCommands {
    float    tgtLMms, tgtRMms;     // per-wheel velocity setpoints (mm/s) — the PID reads these
    int8_t   pwmL, pwmR;           // last PWM written (-100..100), for telemetry/inspection
    int16_t  digitalOut[4];        // 0/1 to drive, -1 = unmanaged (left to direct P commands)
    uint16_t analogOut[4];         // 0..1023
    bool     digitalDirty[4], analogDirty[4];   // set by commands, cleared once pushed
};

// ---- 2. INPUT: snapshot of everything read over I2C / GPIO ----
struct HardwareState {
    float    encLMm, encRMm;       ValueSet enc;     // lag 0; owned by control task
    float    velLMms, velRMms;                       // derived during the encoder read
    float    poseX, poseY, poseHrad;  ValueSet pose; // odometry-predict task writes these
    int16_t  otosX, otosY, otosH;  ValueSet otos;     // default lag 100 ms
    uint16_t line[4];              ValueSet lineVS;   // default lag 50 ms
    uint16_t colorR,colorG,colorB,colorC;  ValueSet colorVS;  // default lag 100 ms
    int16_t  digitalIn[4], analogIn[4];     ValueSet portsVS; // default lag 50 ms
};

// ---- 3. TARGET: where the robot is driving ----
struct TargetState {
    uint8_t  mode;               // DriveMode (IDLE/STREAMING/TIMED/DISTANCE/GO_TO)
    float    targetXWorld, targetYWorld, targetSpeedMms;   // GO_TO goal (world frame)
    int32_t  distanceTargetMm;   // D-mode
    uint32_t deadlineMs;         // T-mode / streaming watchdog
    // captured reply sink so async completions return to the originating channel
    void   (*replyFn)(const char*, void*);   void* replyCtx;   char corrId[16];
};

struct RobotStateContainer { MotorCommands commands; HardwareState inputs; TargetState target; };
```

`Robot` owns one `RobotStateContainer` and exposes it. A `defaultInputs()` helper seeds the
per-set `lagMs` defaults (mirroring `defaultRobotConfig()`); the lags become runtime-tunable
via new `CFG_I` registry entries (`lag.otos`, `lag.line`, `lag.color`, `lag.ports`) in
`source/app/CommandProcessor.cpp`. Setting any lag to `0` forces that set to refresh every loop.

**Authoritative-state consequence (the rewrite):** the subsystems stop owning their own copies
and instead operate on these structs. Control laws and integration math are **reused
verbatim** — only *where the state lives* changes:

- `VelocityController` stays as-is (pure PI+FF with its own integrator — that is controller
  state, not robot state).
- `MotorController` is slimmed to orchestration: read setpoints from `commands.tgt*Mms`,
  read measured `vel*Mms`/`enc*Mm` from `inputs`, run the two `VelocityController`s, write
  `commands.pwm*` and push to `Motor::setSpeed`. Its private encoder/velocity caches move
  into `HardwareState`. The control task's own *previous-encoder + previous-time* snapshot
  (needed to differentiate velocity) lives in the control-task object — it is intermediate
  compute state, not robot state.
- `Odometry` keeps its midpoint `predict()` and complementary `correct()` math, but reads
  encoder values and writes `poseX/poseY/poseHrad` in `HardwareState`. It keeps its own
  prev-encoder snapshot (separate from the control task's, since the two tasks run at
  different cadences).
- `DriveController` keeps the S/T/D/G machines and the GO_TO pursuit/ramp math, but reads
  pose/encoders from `inputs` + the goal from `target`, and writes `commands.tgt*Mms`.

---

## The main loop — class `LoopScheduler`

New files `source/control/LoopScheduler.{h,cpp}`. It holds a reference to `Robot`, the
`CommandProcessor`, `MicroBit&`, the task table, and a round-robin cursor. It owns the
reply-sink adapters (`serialReply`/`radioReply`) that move out of `main.cpp`.

```cpp
struct Task {
    const char* name;
    uint32_t    periodMs;     // due cadence (== the value-set's lagMs for sensor tasks; 0 = always)
    uint32_t    lastRunMs;
    uint32_t    estCostMs;    // conservative worst-case wall cost — used to gate the start
    bool      (*due)(LoopScheduler&, uint32_t now);   // default: now - lastRun >= periodMs
    void      (*run)(LoopScheduler&, uint32_t now);
};
```

`estCostMs` comes from measured I2C timing. With split-phase encoder I/O (see below) the
**control task drops to < 1 ms** (no busy-wait), so a 10 ms period leaves ~9 ms of budget:
OTOS ≈ 2 ms, line ≈ 1 ms, color `pollRGBC` ≈ 1 ms (already non-blocking), ports < 1 ms,
telemetry ≈ 2 ms, comms/odometry/drive-advance < 1 ms each (no I2C).

The **control task is special** (not in the rotated table): it always runs first each
iteration. The remaining tasks live in a single list scanned from a persistent cursor.

```
controlDeadline = now + controlPeriodMs
cursor = 0

loop forever:
    now = systemTime()

    // ---- HARD TASK, always first: collect last encoder + PID + PWM write ----
    controlCollectAndDrive(now)         // split-phase COLLECT + velocity + PID + setSpeed
    controlDeadline = now + controlPeriodMs

    // ---- LOW-PRIORITY SWEEP (round-robin from cursor for fairness) ----
    //   These run AFTER the motor collect/PWM and BEFORE the next encoder request,
    //   so any sensor I2C here is outside the motor's pending-read window.
    swept = 0
    while swept < N:
        i = cursor;  cursor = (cursor + 1) % N;  swept++
        t = table[i]
        now = systemTime()

        // budget gate BEFORE starting: don't launch an I2C read that would overrun control
        if now + t.estCostMs > controlDeadline:  break
        if not t.due(*this, now):                continue   // skip; cursor already advanced

        t.run(*this, now);  t.lastRunMs = now

        // post-task re-check: if control is due, bail back to the top (PID is the metronome)
        if systemTime() >= controlDeadline:      break

    // ---- fire the NEXT wheel's encoder request, then sleep (delay ripens during sleep) ----
    controlFireRequest()                // alternates L/R; LAST I2C op before idle
    now = systemTime()
    if now < controlDeadline:  uBit.sleep(controlDeadline - now)   // the program's only sleep
```

Behavior that falls out of this and matches the requested semantics:

- **Control first, every iteration** — it sets the cadence via `controlDeadline`.
- **Due-check per task** via `ValueSet.lagMs`; not-due tasks are skipped (cursor still advances).
- **Budget gate before starting** prevents launching a multi-ms I2C read that would miss the
  control deadline (this is what makes "fit work into leftover time" safe).
- **Post-task re-check + break** returns to the control task promptly.
- **Persistent cursor** = "continue from that section" — when we bail to service PID, the next
  sweep resumes at the next task, so no task is permanently starved.
- Cheap, always-due tasks (comms-in, odometry-predict, drive-advance; `periodMs` 0, sub-ms
  cost) effectively run every sweep; expensive I2C reads rotate through the leftover window.

---

## Task table → existing code

| Priority order | period / lag | calls (math reused) |
|---|---|---|
| **control** (special, first) | `controlPeriodMs` | split-phase encoder I/O (see below) → compute velocity → `VelocityController::update` ×2 → `Motor::setSpeed`; write `enc*/vel*/pwm*` into the structs. **Nothing else.** Cost < 1 ms (no busy-wait). |
| comms-in | 0 | drain `SerialPort::readLine` + `Radio::poll` → `CommandProcessor::process` (mutates `commands`/`target`/config). Captures the active reply sink. |
| drive-advance | 0 | the S/T/D/G + GO_TO pursuit logic lifted out of today's `controlTick`: reads `inputs.pose`/`enc` + `target`, writes `commands.tgt*Mms`, emits completion events. |
| odometry-predict | 0 (or small) | `Odometry::predict` over the encoder delta in `inputs`, writes `inputs.pose*`. |
| otos-correct | 100 (`otos.lagMs`) | `OtosSensor::getPositionRaw` → LSB conversion → `Odometry::correct` (block lifted out of `controlTick`). |
| line-read | 50 | `LineSensor::readValues(inputs.line)`. |
| color-read | 100 | `ColorSensor::pollRGBC(...)` (non-blocking). |
| ports-read | 50 | `PortIO::readDigital/readAnalog` → `inputs.digitalIn/analogIn`. |
| telemetry-emit | `tlmPeriodMs` | existing TLM assembly, but **reads from `inputs`** instead of re-doing line/color/pose I2C — removes the duplicate reads (and the duplicate-read hazard) in `Robot::telemetryTick`. |

**Event/reply routing simplifies.** With no fiber boundary, the EVT ring buffer is no longer
needed for safety — `drive-advance` runs in the same thread as comms, so completions
(`done T/D/G`, `safety_stop`) are **emitted inline** via the reply sink captured in
`TargetState.replyFn/replyCtx`, preserving reply-to-originating-channel. The ring buffer and
`enqueueEvt`/`drainEvents` are removed.

---

## I2C: split-phase, no busy-wait

`Motor::readEncoderRaw` today brackets the `0x46` write with **4 ms before + 4 ms after** and
busy-waits both (`readSpeedRaw`: 4 ms + 8 ms). The delays are **pure chip timing** — the
"before" lets the bus idle, the "after" lets the chip prepare its response. The *atomicity*
rationale in those comments is explicitly a **two-fiber artifact** (busy-wait was chosen over
`fiber_sleep` so the comms fiber couldn't issue a competing write mid-transaction). With one
loop, that reason is gone — and blocking the loop for 8–16 ms per tick is exactly what we want
to eliminate.

**Replace the blocking transaction with a split-phase, non-blocking state machine.** The loop's
own period supplies the delay for free; the chip buffers only one outstanding request (the
command carries `motorId`), so we **alternate wheels** — one encoder serviced per control
iteration:

- **Collect** (top of control task): read back the 4 bytes for the wheel whose request was
  fired at the end of the *previous* iteration. The request→collect gap is the idle-sleep +
  loop overhead — one full control period (≥10 ms) ≫ the 4 ms the chip needs.
- Compute that wheel's velocity from its own ~2-period delta; run **both** wheels' PID
  (zero-order-hold on the measurement that didn't refresh this tick); write PWM (`Motor::setSpeed`).
- **Request** (end of control task): fire the `0x46` command for the *other* wheel, stamp the
  time, then idle-sleep.

Per-wheel sample rate ≈ `controlPeriodMs × 2` (~50 Hz at 10 ms — better than today's ~40 Hz
serialized rate), with the busy-wait gone the control task costs < 1 ms, freeing ~9 ms of every
10 ms period for sensor tasks.

**Ordering rule that keeps the pending-read window safe** (whether or not the chip tolerates
other-address traffic — an open hardware question we sidestep rather than depend on):

> **collect + PWM write at the top → sensor I2C tasks in the middle → fire the next encoder
> request *last*, just before the idle sleep.**

This guarantees by construction that no other I2C transaction occurs inside the motor's
request→collect window. `Motor.cpp` changes: `readEncoderRaw` splits into
`requestEncoder(wheel)` (write + timestamp) and `collectEncoder(wheel)` (read-back, no delay);
the busy-wait loops are deleted. The chip speed register (`0x47`) stays disabled — encoder-delta
velocity over the ~2-period window is sufficient; it can get the same split-phase treatment
later if needed. **Bench-confirm** the alternation cadence and that velocity/encoders stay clean
with sensor tasks running in the middle of the window.

---

## File-by-file changes

**New**
- `source/control/RobotState.h` — the three structs + `RobotStateContainer` + `defaultInputs()`.
- `source/control/LoopScheduler.{h,cpp}` — `Task`, the table, the loop algorithm, reply sinks.

**Modified**
- `source/main.cpp` — shrinks to: `uBit.init()`, construct `Robot` + `CommandProcessor` +
  `LoopScheduler`, emit HELLO, `sched.run()`. Remove `controlFiberFn`, `gRobot`,
  `create_fiber`, both `uBit.sleep` loops.
- `source/robot/Robot.{h,cpp}` — own the `RobotStateContainer`; replace `controlTick`/
  `telemetryTick` framing with granular task entry points; telemetry reads from `inputs`.
- `source/control/MotorController.{h,cpp}` — slim to orchestration over the structs;
  encoder/velocity caches move to `HardwareState`.
- `source/control/Odometry.{h,cpp}` — `predict`/`correct` operate on `inputs.pose*`.
- `source/control/DriveController.{h,cpp}` — split into the drive-advance task + otos-correct
  task; read `inputs`/`target`, write `commands`; **remove EVT ring**, emit inline.
- `source/types/Config.h` — keep `controlPeriodMs`; lag defaults live in `defaultInputs()`.
- `source/app/CommandProcessor.cpp` — add `CFG_I` registry entries for the lag settings.
- `source/hal/Motor.{h,cpp}` — split `readEncoderRaw` into `requestEncoder(wheel)` +
  `collectEncoder(wheel)`; **delete the busy-wait loops**. (`0x47` speed read stays disabled.)

---

## Verification

**Pytest suite** (protocol unchanged — the regression gate). Run all of `tests/` via the
project venv (`uv run`), especially: `test_tlm_stream.py` (telemetry now reads snapshots —
set `lag.line 0`/`lag.color 0` to force fresh per-loop reads and confirm freshness);
`test_motion_verbs_v2.py`, `test_vw_command.py`, `test_pursuit_arc_steering.py`,
`test_saturation_wiring.py` (S/T/D/G + inline EVT completions route to the right channel);
`test_odometry_midpoint.py`, `test_otos_fusion.py` (predict every cycle, correct at otos lag);
`test_readspeed_and_get_vel.py` (velocity/`GET VEL`).

**Hardware bench** (robot on the stand — safe to drive; sensors + wheels + encoders):
1. Clean build (`--clean`) and flash; confirm HELLO banner + boot icon.
2. `S 200 200` → both wheels spin, `GET VEL` plausible, encoders advance. Measure effective
   control rate (temporary tick-count telemetry field) — should be ≥ the current ~40 Hz.
3. Streaming watchdog: `S 200 200`, stop sending → `EVT safety_stop` after `sTimeoutMs`,
   motors stop (validates inline event emission).
4. `T`, `D`, `G` → completion EVT on the originating channel; pose converges under OTOS correction.
5. Lag tuning: set `lag.line 0`/`lag.color 0` and confirm fresh per-loop sensor values; set
   them high (500 ms) and confirm sensors update slowly while the control rate is unaffected —
   this directly validates the budget gate and round-robin fairness.
6. Stress: stream commands rapidly over radio while driving — control rate must hold steady
   (PID stays the metronome) and commands are still serviced.

This is a CLASI project; once the design is agreed, the execution vehicle is a new sprint
(architecture update + sequenced tickets) created via the CLASI MCP tools rather than ad-hoc edits.
