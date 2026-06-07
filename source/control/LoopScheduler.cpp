#include "LoopScheduler.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "Communicator.h"
#include "SerialPort.h"
#include "Radio.h"
#include "RobotState.h"

// ---------------------------------------------------------------------------
// Reply-sink adapters — moved from main.cpp to LoopScheduler (014-006).
//
// These thin static functions bridge the (const char*, void*) ReplyFn
// signature to the HAL send() methods on SerialPort and Radio.
// ---------------------------------------------------------------------------

static void serialReply(const char* msg, void* ctx)
{
    static_cast<SerialPort*>(ctx)->send(msg);
}

static void radioReply(const char* msg, void* ctx)
{
    static_cast<Radio*>(ctx)->send(msg);
}

// ---------------------------------------------------------------------------
// Default due() predicate — shared by all tasks that use the standard
// cadence check: due when periodMs == 0 (always) or when
// now - lastRunMs >= periodMs.
// ---------------------------------------------------------------------------

static bool defaultDue(Task& task, uint32_t now)
{
    if (task.periodMs == 0) return true;
    return (now - task.lastRunMs) >= task.periodMs;
}

// ---------------------------------------------------------------------------
// Task run() functions — one per low-priority task.
// Each is a plain static function matching void(*)(LoopScheduler&, uint32_t).
// ---------------------------------------------------------------------------

// comms-in: drain serial and radio command queues, dispatch to CommandProcessor.
// Captures the active reply sink (activeFn/activeCtx) on the scheduler.
static void runCommsIn(LoopScheduler& sched, uint32_t /*now*/)
{
    CommandProcessor& cmd    = sched.cmd();
    SerialPort&       serial = sched.comm().serial();
    Radio&            radio  = sched.comm().radio();

    char buf[512];

    while (serial.readLine(buf, sizeof(buf))) {
        sched.activeFn  = serialReply;
        sched.activeCtx = &serial;
        cmd.process(buf, serialReply, &serial);
    }

    while (radio.poll(buf, sizeof(buf))) {
        sched.activeFn  = radioReply;
        sched.activeCtx = &radio;
        cmd.process(buf, radioReply, &radio);
    }
}

// drive-advance: advance the S/T/D/G drive-mode state machines; emits
// EVT completions inline via the captured reply sink in TargetState.
static void runDriveAdvance(LoopScheduler& sched, uint32_t now)
{
    sched.robot().driveAdvance(now);
}

// odometry-predict: apply midpoint dead-reckoning from enc{L,R}Mm into
// poseX/Y/Hrad in HardwareState.
static void runOdometryPredict(LoopScheduler& sched, uint32_t /*now*/)
{
    sched.robot().odometryPredict();
}

// otos-correct: read OTOS hardware and apply complementary fusion correction.
static void runOtosCorrect(LoopScheduler& sched, uint32_t now)
{
    sched.robot().otosCorrect(now);
}

// line-read: delegate to Robot::lineRead() task entry point (014-007).
static void runLineRead(LoopScheduler& sched, uint32_t /*now*/)
{
    sched.robot().lineRead();
}

// color-read: delegate to Robot::colorRead() task entry point (014-007).
static void runColorRead(LoopScheduler& sched, uint32_t /*now*/)
{
    sched.robot().colorRead();
}

// ports-read: delegate to Robot::portsRead() task entry point (014-007).
static void runPortsRead(LoopScheduler& sched, uint32_t /*now*/)
{
    sched.robot().portsRead();
}

// telemetry-emit: assemble and send the unified TLM frame from _state.inputs
// snapshots (no direct sensor I2C calls — reads from HardwareState written by
// lineRead, colorRead, controlCollect task entry points).
static void runTelemetryEmit(LoopScheduler& sched, uint32_t now)
{
    sched.robot().telemetryEmit(now, sched.activeFn, sched.activeCtx);
}

// ---------------------------------------------------------------------------
// LoopScheduler constructor
//
// Initialises the task table with name / periodMs / lastRunMs / estCostMs /
// due / run for each of the 8 low-priority tasks.
// Period values are seeded from the robot config at construction time; they
// can be changed at runtime by the command processor (SET lag.*).
// ---------------------------------------------------------------------------

LoopScheduler::LoopScheduler(Robot& robot, CommandProcessor& cmd,
                             Communicator& comm, MicroBit& uBit)
    : activeFn(nullptr),
      activeCtx(nullptr),
      _robot(robot),
      _cmd(cmd),
      _comm(comm),
      _uBit(uBit),
      _cursor(0),
      _pendingWheel(0),
      _controlDeadline(0),
      _controlRuns(0),
      _controlTotalUs(0),
      _loopRuns(0),
      _loopTotalUs(0),
      _loopWorkTotalUs(0)
{
    const RobotConfig& cfg = robot.config();

    // -----------------------------------------------------------------------
    // Task table — 8 low-priority tasks in priority order.
    //
    // periodMs: 0 = run every iteration (always-due); otherwise minimum
    //           interval in ms between calls.
    // estCostMs: conservative worst-case wall cost used for budget gating.
    //   comms-in:        < 1 ms (no I2C)
    //   drive-advance:   < 1 ms (no I2C)
    //   odometry-predict:< 1 ms (pure math)
    //   otos-correct:    ~ 2 ms (one I2C read)
    //   line-read:       ~ 1 ms (one I2C read)
    //   color-read:      ~ 1 ms (non-blocking poll)
    //   ports-read:      < 1 ms (GPIO)
    //   telemetry-emit:  ~ 2 ms (snprintf + serial send)
    // -----------------------------------------------------------------------

    _table[0] = {
        "comms-in",
        0,                  // always due
        0,                  // lastRunMs (never run yet)
        0,                  // estCostMs < 1 ms
        defaultDue,
        runCommsIn
    };

    _table[1] = {
        "drive-advance",
        0,                  // always due
        0,
        0,
        defaultDue,
        runDriveAdvance
    };

    _table[2] = {
        "odometry-predict",
        0,                  // always due
        0,
        0,
        defaultDue,
        runOdometryPredict
    };

    _table[3] = {
        "otos-correct",
        cfg.lagOtosMs,      // default 100 ms
        0,
        2,                  // ~2 ms I2C read
        defaultDue,
        runOtosCorrect
    };

    _table[4] = {
        "line-read",
        cfg.lagLineMs,      // default 50 ms
        0,
        1,                  // ~1 ms I2C read
        defaultDue,
        runLineRead
    };

    _table[5] = {
        "color-read",
        cfg.lagColorMs,     // default 100 ms
        0,
        1,                  // ~1 ms poll
        defaultDue,
        runColorRead
    };

    _table[6] = {
        "ports-read",
        cfg.lagPortsMs,     // default 50 ms
        0,
        0,                  // < 1 ms GPIO
        defaultDue,
        runPortsRead
    };

    _table[7] = {
        "telemetry-emit",
        (uint32_t)cfg.tlmPeriodMs,   // 0 = off; set non-zero by STREAM command
        0,
        2,                            // ~2 ms snprintf + serial write
        defaultDue,
        runTelemetryEmit
    };

    // Defaults for the run-control flags + timing stats on every task.
    // (The 6-field aggregate initialisers above leave these zero-initialised;
    // set the intended defaults here: armed, re-armed every pass, not one-shot.)
    for (int i = 0; i < kNumTasks; ++i) {
        _table[i].run         = true;
        _table[i].run_always  = true;
        _table[i].run_once    = false;
        _table[i].runs        = 0;
        _table[i].totalTimeUs = 0;
    }

    // All tasks default to run=true.  Disabling a sensor is done by commenting
    // its begin() call in main() (its reads then skip via is_initialized());
    // the "DBG LOOP <x> 0/1" command still toggles tasks at runtime.
}

// ---------------------------------------------------------------------------
// controlCollect — private, wraps Robot::controlCollectSplitPhase.
//
// In the proper split-phase path (when _pendingWheel != 0), this collects
// the encoder from the wheel that was requested at the end of the PREVIOUS
// iteration, then runs PID and writes PWM.
//
// On the first iteration (_pendingWheel == 0), no request has been fired
// yet, so the collect is skipped (only PID runs on zero-delta inputs).
// ---------------------------------------------------------------------------

void LoopScheduler::controlCollect(uint32_t now_ms)
{
    _robot.controlCollectSplitPhase(now_ms, _pendingWheel);
}

// ---------------------------------------------------------------------------
// _advancePendingWheel — advance the L/R alternation cursor.
//
// Called at the END of each tick (after the sensor sweep) to set up which
// wheel will be atomically read at the TOP of the next tick.
//
// Alternation:
//   _pendingWheel == 0 (first iter): advance to 1 (left)
//   _pendingWheel == 1 (left just read): advance to 2 (right)
//   _pendingWheel == 2 (right just read): advance to 1 (left)
// ---------------------------------------------------------------------------

void LoopScheduler::_advancePendingWheel()
{
    if (_pendingWheel == 1) {
        _pendingWheel = 2;
    } else {
        // Either first iteration (0) or right was just read (2): next = left.
        _pendingWheel = 1;
    }
}

// ---------------------------------------------------------------------------
// controlFireRequest — retained for API compatibility.
//
// The encoder request is now issued atomically at the TOP of the tick inside
// controlCollect(), so this method is no longer called by run().  Retained
// to avoid breaking Robot::controlFireRequest() linkage.
// ---------------------------------------------------------------------------

void LoopScheduler::controlFireRequest()
{
    if (_pendingWheel == 1) {
        _robot.controlFireRequest(2);
        _pendingWheel = 2;
    } else {
        _robot.controlFireRequest(1);
        _pendingWheel = 1;
    }
}

// ---------------------------------------------------------------------------
// run_tasks — the production cooperative main loop. Never returns.
//
// Iteration structure (fix[014]: atomic per-tick encoder read):
//
//   1. HARD TASK (always first):
//      a. Atomic encoder read for the pending wheel, using full vendor timing:
//           4ms pre-write idle → requestEncoder(wheel) → 4ms post-write settle
//           → collectEncoder(wheel).
//         Both delays match the sprint 013 readEncoderRaw() pattern (vendor
//         pxt-nezha2 readAngle()). Pre-write allows the bus to idle; post-write
//         allows the chip to prepare its response. Cost: ~8 ms/tick.
//      b. PID runs; PWM written to motors.
//      c. Set controlDeadline = now + controlPeriodMs.
//
//   2. LOW-PRIORITY SWEEP (persistent round-robin cursor):
//      For each task starting from _cursor (mod kNumTasks):
//        a. Budget gate: if (now + task.estCostMs > controlDeadline) break.
//        b. Due check:   if (!task.due(task, now)) continue (cursor advances).
//        c. Run task; set task.lastRunMs = now.
//        d. Post-task deadline re-check: if (systemTime() >= controlDeadline) break.
//      The cursor persists — next iteration resumes where this one left off.
//
//   3. ADVANCE WHEEL CURSOR:
//      _advancePendingWheel() sets up which wheel will be read next tick (L/R alt).
//      No I2C here — the request is now issued at the TOP of the tick.
//
//   4. IDLE SLEEP:
//      uBit.sleep(controlDeadline - now) — the program's only sleep.
//      With the 8 ms encoder cost moved into step 1, the idle sleep is ~2 ms
//      at a 10 ms control period.  Raise controlPeriodMs if the sweep needs room.
// ---------------------------------------------------------------------------

void LoopScheduler::run_tasks()
{
    // Seed controlDeadline so the first iteration's sleep fires promptly.
    _controlDeadline = _uBit.systemTime();

    while (true) {
        uint32_t now = _uBit.systemTime();

        // ------------------------------------------------------------------
        // 1. HARD TASK: collect encoder + PID + PWM write.
        // ------------------------------------------------------------------
        controlCollect(now);

        // Set the deadline for the NEXT control iteration.
        now = _uBit.systemTime();
        _controlDeadline = now + (uint32_t)_robot.config().controlPeriodMs;

        // ------------------------------------------------------------------
        // 2. LOW-PRIORITY SWEEP (round-robin, persistent cursor).
        // ------------------------------------------------------------------
        // Sync task periods from config each iteration so that SET lag.* and
        // STREAM commands take effect without a reboot.
        // _table indices: 3=otos-correct, 4=line-read, 5=color-read, 6=ports-read,
        //                 7=telemetry-emit.
        const RobotConfig& cfg = _robot.config();
        _table[3].periodMs = cfg.lagOtosMs;
        _table[4].periodMs = cfg.lagLineMs;
        _table[5].periodMs = cfg.lagColorMs;
        _table[6].periodMs = cfg.lagPortsMs;
        _table[7].periodMs = (uint32_t)cfg.tlmPeriodMs;

        int swept = 0;
        while (swept < kNumTasks) {
            int idx = _cursor;
            _cursor = (_cursor + 1) % kNumTasks;
            swept++;

            Task& t = _table[idx];
            now = _uBit.systemTime();

            // Budget gate: don't start a task that would overrun the control deadline.
            if (now + t.estCostMs > _controlDeadline) {
                break;
            }

            // Due check: skip if not yet due (cursor already advanced above).
            if (!t.due(t, now)) {
                continue;
            }

            // Run the task.
            t.runFn(*this, now);
            t.lastRunMs = now;

            // Post-task deadline re-check: bail if control is due.
            if (_uBit.systemTime() >= _controlDeadline) {
                break;
            }
        }

        // ------------------------------------------------------------------
        // 3. Advance _pendingWheel for the NEXT iteration's collect.
        //    (The encoder read is now done atomically at the TOP of the
        //    tick via request → 4 ms busy-wait → collect, so no separate
        //    fire-request step is needed here.  We only advance the wheel
        //    alternation cursor so controlCollect() knows which wheel to
        //    refresh next tick.)
        // ------------------------------------------------------------------
        _advancePendingWheel();

        // ------------------------------------------------------------------
        // 4. IDLE SLEEP until the control deadline.
        // ------------------------------------------------------------------
        now = _uBit.systemTime();
        if (now < _controlDeadline) {
            _uBit.sleep(_controlDeadline - now);
        }
    }
}

// ---------------------------------------------------------------------------
// _runStep — run one task with the run-flag guard + per-task timing.
//
// Skips entirely if t.run == false.  Otherwise times t.runFn with the
// microsecond timer, accumulates t.runs / t.totalTimeUs (for averaging), and
// — if the task is one-shot (run_once && !run_always) — disarms it (run=false)
// so it won't run again.
// ---------------------------------------------------------------------------

void LoopScheduler::_runStep(Task& t, uint32_t now)
{
    if (!t.run) {
        return;
    }
    uint64_t t0 = system_timer_current_time_us();
    t.runFn(*this, now);
    uint64_t t1 = system_timer_current_time_us();

    t.totalTimeUs += (uint32_t)(t1 - t0);
    t.runs++;
    t.lastRunMs = now;

    if (t.run_once && !t.run_always) {
        t.run = false;   // one-shot: disarm after a single run
    }
}

// ---------------------------------------------------------------------------
// Debug/testing task control (DBG LOOP command).
// ---------------------------------------------------------------------------

bool LoopScheduler::setTaskRun(int idx, bool run)
{
    if (idx < 0 || idx >= kNumTasks) {
        return false;
    }
    _table[idx].run = run;
    return true;
}

const Task* LoopScheduler::taskAt(int idx) const
{
    if (idx < 0 || idx >= kNumTasks) {
        return nullptr;
    }
    return &_table[idx];
}

// ---------------------------------------------------------------------------
// run_all — explicit testing loop. Never returns.
//
// Unlike run_tasks(), this does NOT iterate the table or apply budget/due
// gating.  Every task is called EXPLICITLY, in a visible fixed order, each
// guarded by its Task::run flag and individually timed.  This makes it trivial
// to reorder steps, toggle them (set t.run / t.run_always / t.run_once), and
// read off per-task averages (totalTimeUs / runs).
//
// The control task (split-phase encoder collect → PID → PWM) is special (not a
// Task entry); it always runs first and is timed into _controlRuns/_controlTotalUs.
// ---------------------------------------------------------------------------

void LoopScheduler::run_all()
{
    _controlDeadline = _uBit.systemTime();

    while (true) {
        uint64_t iter0 = system_timer_current_time_us();   // loop-period start
        uint32_t now = _uBit.systemTime();

        // --- CONTROL TASK (always first; the metronome) — timed -------------
        {
            uint64_t c0 = system_timer_current_time_us();
            controlCollect(now);
            uint64_t c1 = system_timer_current_time_us();
            _controlTotalUs += (uint32_t)(c1 - c0);
            _controlRuns++;
        }
        now = _uBit.systemTime();
        _controlDeadline = now + (uint32_t)_robot.config().controlPeriodMs;

        // (Sensor detection is done once in main() before this loop starts.)

        // --- LOW-PRIORITY TASKS — explicit, in order, each guarded + timed --
        // Reorder / comment-out / toggle (_table[i].run = false) freely.
        // FIXME why is `now` not updated between steps here?  Should it be?  (Some steps use it, some don't.)
        // it's very stale by the time we get to step 7/8, which use it for telemetry timestamps.  Maybe update it at the top of each step?
        _runStep(_table[0], now);   // comms-in
        _runStep(_table[1], now);   // drive-advance (watchdog / ESC stop)
        _runStep(_table[2], now);   // odometry-predict
        _runStep(_table[3], now);   // otos-correct
        _runStep(_table[4], now);   // line-read
        _runStep(_table[5], now);   // color-read
        _runStep(_table[6], now);   // ports-read
        _runStep(_table[7], now);   // telemetry-emit

        // --- advance L/R wheel alternation for next tick's collect ----------
        _advancePendingWheel();

        // Work time for this iteration (control + sweep, before idle sleep).
        uint64_t work1 = system_timer_current_time_us();
        _loopWorkTotalUs += (uint32_t)(work1 - iter0);

        // --- idle sleep until the control deadline --------------------------
        now = _uBit.systemTime();
        if (now < _controlDeadline) {
            _uBit.sleep(_controlDeadline - now);
        }

        // Full iteration period (incl. idle sleep) = the loop cycle time.
        uint64_t iter1 = system_timer_current_time_us();
        _loopTotalUs += (uint32_t)(iter1 - iter0);
        _loopRuns++;
    }
}

// ---------------------------------------------------------------------------
// run_blocks — the straightforward main loop. Never returns.
//
// No task table, no _runStep, no round-robin cursor, no budget gating. Every
// subsystem is an explicit BLOCK right here in the loop body, so you can read
// the whole schedule top to bottom. Each block is gated by two things:
//
//   1. an ENABLE FLAG (en*) — set false to turn that block off entirely; and
//   2. a TIME CHECK — using a SIGNED delta (int32_t)(now - last) so a uint32
//      millisecond wrap/inversion can never make a block fire ~4.3e9 ms "late"
//      or never (this is the watchdog-underflow lesson — never plain-compare two
//      uint32 ms stamps).
//
// Built from the WedgeTest recipe that runs wedge-free on the bench:
//   - the CONTROL block (read BOTH encoders, M1/right first → velocity → per-
//     wheel PID → setSpeed) runs EVERY iteration as the metronome;
//   - the idle sleep paces the whole loop to a fixed controlPeriodMs deadline;
//   - the timed sensor/telemetry blocks run only when their interval elapses.
//
// To turn a subsystem on/off, flip its en* flag below. To change a rate, edit
// the matching cfg.lag*Ms / cfg.tlmPeriodMs (also live-settable via SET).
// ---------------------------------------------------------------------------

void LoopScheduler::run_blocks()
{
    // ---- ENABLE FLAGS: is each block turned on? -----------------------------
    bool enControl = true;   // read encoders + run PID + write motors (metronome)
    bool enComms   = true;   // drain serial + radio command queues
    bool enDrive   = true;   // advance S/T/D/G drive state machine + S-watchdog
    bool enOdom    = true;   // dead-reckon pose from the encoders
    bool enOtos    = true;   // OTOS pose read + complementary fusion (timed)
    bool enLine    = true;   // line sensor read (timed)
    bool enColor   = true;   // colour sensor read (timed)
    bool enPorts   = true;   // port-IO / GPIO read (timed)
    bool enTlm     = true;   // assemble + send the TLM telemetry frame (timed)

    // ---- per-block last-run timestamps for the timed blocks -----------------
    // Seed to NOW (not 0): with 0, (now - 0) >= period is true for every block
    // on the first iteration, so they'd ALL fire on tick 1 alongside the control
    // read and overrun the period (there is no budget gate here). Seeding to now
    // makes each wait a full period first. The small descending offsets phase-
    // spread the first runs so the timed blocks don't keep coming due together.
    uint32_t t0 = _uBit.systemTime();
    uint32_t lastOtos  = t0;
    uint32_t lastLine  = t0 - 10;
    uint32_t lastColor = t0 - 20;
    uint32_t lastPorts = t0 - 30;
    uint32_t lastTlm   = t0 - 40;

    uint32_t controlDeadline = 0;

    while (true) {
        const RobotConfig& cfg = _robot.config();
        uint32_t now = _uBit.systemTime();

        // ===== CONTROL: read BOTH encoders (M1 first) → PID → setSpeed =======
        // Runs every iteration; the idle sleep at the bottom paces the loop to
        // cfg.controlPeriodMs. This is the WedgeTest-proven read-both pattern
        // (Robot::controlCollectSplitPhase): right/M1 encoder first, then left,
        // with outlier rejection, then the per-wheel velocity PID writes PWM.
        if (enControl) {
            controlCollect(now);
        }
        now = _uBit.systemTime();
        controlDeadline = now + (uint32_t)cfg.controlPeriodMs;

        // ===== COMMS: drain serial + radio command queues (every iteration) ==
        if (enComms) {
            now = _uBit.systemTime();
            runCommsIn(*this, now);
        }

        // ===== DRIVE: advance the drive state machine + S-watchdog ===========
        if (enDrive) {
            now = _uBit.systemTime();
            _robot.driveAdvance(now);
        }

        // ===== ODOMETRY: dead-reckon pose from the latest encoder deltas =====
        if (enOdom) {
            _robot.odometryPredict();
        }

        // ===== OTOS: timed I2C pose read + fusion ============================
        now = _uBit.systemTime();
        if (enOtos && cfg.lagOtosMs > 0 &&
            (int32_t)(now - lastOtos) >= (int32_t)cfg.lagOtosMs) {
            _robot.otosCorrect(now);
            lastOtos = now;
        }

        // ===== LINE: timed I2C read =========================================
        now = _uBit.systemTime();
        if (enLine && cfg.lagLineMs > 0 &&
            (int32_t)(now - lastLine) >= (int32_t)cfg.lagLineMs) {
            _robot.lineRead();
            lastLine = now;
        }

        // ===== COLOUR: timed read ===========================================
        now = _uBit.systemTime();
        if (enColor && cfg.lagColorMs > 0 &&
            (int32_t)(now - lastColor) >= (int32_t)cfg.lagColorMs) {
            _robot.colorRead();
            lastColor = now;
        }

        // ===== PORTS: timed GPIO read =======================================
        now = _uBit.systemTime();
        if (enPorts && cfg.lagPortsMs > 0 &&
            (int32_t)(now - lastPorts) >= (int32_t)cfg.lagPortsMs) {
            _robot.portsRead();
            lastPorts = now;
        }

        // ===== TELEMETRY: timed TLM frame emit ==============================
        now = _uBit.systemTime();
        if (enTlm && cfg.tlmPeriodMs > 0 &&
            (int32_t)(now - lastTlm) >= (int32_t)cfg.tlmPeriodMs) {
            _robot.telemetryEmit(now, activeFn, activeCtx);
            lastTlm = now;
        }

        // ===== IDLE SLEEP until the control deadline ========================
        now = _uBit.systemTime();
        if ((int32_t)(controlDeadline - now) > 0) {
            _uBit.sleep(controlDeadline - now);
        }
    }
}

// ---------------------------------------------------------------------------
// resetStats — zero all timing counters so DBG LOOP reports a fresh window.
// ---------------------------------------------------------------------------
void LoopScheduler::resetStats()
{
    _controlRuns = 0;
    _controlTotalUs = 0;
    _loopRuns = 0;
    _loopTotalUs = 0;
    _loopWorkTotalUs = 0;
    for (int i = 0; i < kNumTasks; ++i) {
        _table[i].runs = 0;
        _table[i].totalTimeUs = 0;
    }
}
