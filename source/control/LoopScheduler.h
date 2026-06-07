#pragma once
#include "MicroBit.h"
#include "Protocol.h"

// Forward declarations to avoid pulling in the full header graph.
class Robot;
class CommandProcessor;
class Communicator;

// ---------------------------------------------------------------------------
// Task — descriptor for one slot in the cooperative scheduler's task table.
//
// Fields:
//   name       : human-readable name for diagnostics.
//   periodMs   : minimum interval between calls (0 = run every iteration).
//   lastRunMs  : system time (ms) of the most recent run.
//   estCostMs  : conservative worst-case wall-clock cost in ms; used as the
//                budget gate before starting the task (if
//                now + estCostMs > controlDeadline, don't start it).
//   due        : returns true when the task is eligible to run (default:
//                periodMs==0, or now - lastRunMs >= periodMs).
//   runFn      : executes the task body (was 'run'; renamed so the 'run' bool
//                flag below is unambiguous).
//
//   run        : per-step enable gate. run_all() runs this task iff run==true.
//   run_always : default true  — re-armed every pass (never auto-disabled).
//   run_once   : default false — after one run, disarm (run := false) unless
//                run_always is also set.
//   runs       : number of times runFn has been invoked (for averaging).
//   totalTimeUs: accumulated wall time across all runs, in microseconds.
//                Average per-run cost = totalTimeUs / runs.
// ---------------------------------------------------------------------------
struct Task {
    const char* name;
    uint32_t    periodMs;
    uint32_t    lastRunMs;
    uint32_t    estCostMs;
    bool      (*due)(struct Task& task, uint32_t now);
    void      (*runFn)(class LoopScheduler& sched, uint32_t now);

    // run-control flags (used by run_all() via _runStep)
    bool        run;
    bool        run_always;
    bool        run_once;

    // per-task timing stats
    uint32_t    runs;
    uint32_t    totalTimeUs;
};

// ---------------------------------------------------------------------------
// LoopScheduler — single cooperative main loop for the robot firmware.
//
// Replaces the two-fiber (control fiber + comms fiber) architecture with a
// single cooperative priority-task loop:
//
//   1. HARD TASK (always first): split-phase encoder COLLECT → velocity →
//      per-wheel PID → Motor::setSpeed. Sets controlDeadline.
//   2. LOW-PRIORITY SWEEP (round-robin, persistent cursor): for each task
//      starting from the cursor, check the budget gate, check due(), run;
//      re-check deadline after each run; break when over budget or deadline.
//   3. ENCODER REQUEST: fire the next wheel's encoder request (L/R alternating).
//      This is the LAST I2C operation before idle — keeps the motor's pending-
//      read window free of other I2C.
//   4. IDLE SLEEP: uBit.sleep(controlDeadline - now) — the program's only sleep.
//
// The task table contains the eight low-priority tasks in priority order:
//   comms-in, drive-advance, odometry-predict, otos-correct,
//   line-read, color-read, ports-read, telemetry-emit.
//
// Reply-sink adapters (serialReply, radioReply) are defined in
// LoopScheduler.cpp, moved from main.cpp.
//
// Ordering rule (maintained by construction):
//   collect + PWM at the top → all sensor-I2C tasks in the middle of the
//   sweep → encoder request fired last. This guarantees no I2C transaction
//   occurs inside the motor's pending-read window.
//
// Construction:
//   LoopScheduler sched(robot, cmd, comm, uBit);
//   sched.run_tasks();   // never returns (or run_all() for testing)
// ---------------------------------------------------------------------------
class LoopScheduler {
public:
    LoopScheduler(Robot& robot, CommandProcessor& cmd, Communicator& comm, MicroBit& uBit);

    // Production cooperative main loop: priority task table, budget gating,
    // round-robin sweep. Never returns. (Was run().)
    void run_tasks();

    // Explicit testing loop: every task is called explicitly, in a visible
    // fixed order, each guarded by its Task::run flag and individually timed
    // (Task::runs / Task::totalTimeUs). Easy to reorder / toggle by hand.
    // Never returns.
    void run_all();

    // run_blocks — the straightforward, fully-inlined main loop. No task table,
    // no _runStep, no budget/cursor machinery. Every subsystem is an explicit
    // block right in the loop body, each gated by (1) a plain on/off enable flag
    // and (2) a signed-delta time check (the watchdog pattern). Built from the
    // WedgeTest recipe: read BOTH encoders (M1 first) + PID every tick as the
    // metronome, idle-sleep to a fixed control-period deadline. Never returns.
    void run_blocks();

    // --- Debug/testing task control (DBG LOOP command) -----------------------
    // Number of low-priority tasks; valid indices are 0..numTasks()-1.
    int  numTasks() const { return kNumTasks; }
    // Enable/disable a task's run flag (takes effect in run_all). Returns false
    // if idx is out of range.
    bool setTaskRun(int idx, bool run);
    // Read-only access to a task entry (name / run / runs / totalTimeUs).
    // Returns nullptr if idx is out of range.
    const Task* taskAt(int idx) const;

    // Control-task (PID metronome) timing — it is not a Task entry.
    uint32_t controlRuns()     const { return _controlRuns; }
    uint32_t controlTotalUs()  const { return _controlTotalUs; }
    // Whole-loop timing: full iteration incl. idle sleep (the cycle period) and
    // work-only (excl. idle sleep). Averages = totalUs / loopRuns.
    uint32_t loopRuns()        const { return _loopRuns; }
    uint32_t loopTotalUs()     const { return _loopTotalUs; }
    uint32_t loopWorkTotalUs() const { return _loopWorkTotalUs; }
    // Zero all timing stats (control, loop, and every task) — for sampling a
    // steady-state window (DBG LOOP RESET).
    void resetStats();

    // ---------------------------------------------------------------------------
    // Accessors used by Task::run() lambdas.
    // ---------------------------------------------------------------------------
    Robot&            robot() { return _robot; }
    CommandProcessor& cmd()   { return _cmd;   }
    Communicator&     comm()  { return _comm;  }
    MicroBit&         uBit()  { return _uBit;  }

    // Active reply sink — updated each time a command is dispatched so that
    // telemetry and event completions go back over the originating channel.
    ReplyFn  activeFn;
    void*    activeCtx;

private:
    Robot&            _robot;
    CommandProcessor& _cmd;
    Communicator&     _comm;
    MicroBit&         _uBit;

    // ---------------------------------------------------------------------------
    // Task table — 8 low-priority tasks in priority order.
    // ---------------------------------------------------------------------------
    static constexpr int kNumTasks = 8;
    Task _table[kNumTasks];

    // Round-robin cursor — persists across iterations for fairness.
    // On each sweep we start at _cursor and advance modulo kNumTasks.
    int _cursor;

    // ---------------------------------------------------------------------------
    // Split-phase encoder state.
    //
    // _pendingWheel: 0 = no request fired yet (first-iteration guard);
    //                1 = left wheel request in flight;
    //                2 = right wheel request in flight.
    //
    // _controlDeadline: system time (ms) by which the next control task
    //                   iteration must begin.
    // ---------------------------------------------------------------------------
    int      _pendingWheel;
    uint32_t _controlDeadline;

    // Control-task timing (the control task is special — not a Task entry, so
    // its stats live here rather than in the table). Average = us/runs.
    uint32_t _controlRuns;
    uint32_t _controlTotalUs;

    // Whole-loop iteration timing.
    uint32_t _loopRuns;
    uint32_t _loopTotalUs;       // full iteration incl. idle sleep (cycle period)
    uint32_t _loopWorkTotalUs;   // work per iteration excl. idle sleep

    // ---------------------------------------------------------------------------
    // Private helpers that implement the split-phase control logic.
    // ---------------------------------------------------------------------------

    // Control task: for the current _pendingWheel, issue the full vendor
    // timing: 4ms pre-write idle → requestEncoder() → 4ms post-write settle →
    // collectEncoder(), all atomic within this tick, then run per-wheel PID and
    // write PWM.  Skips the read if _pendingWheel == 0 (first iteration).
    void controlCollect(uint32_t now_ms);

    // Advance _pendingWheel for the NEXT iteration (L → R → L alternation).
    // Called at the end of each tick after the sensor sweep.
    void _advancePendingWheel();

    // Retained for API compatibility — fires an encoder request for the given
    // wheel via Robot::controlFireRequest().  No longer called by run() since
    // the request is now issued atomically at the top of the tick.
    void controlFireRequest();

    // Run one task with the run-flag guard + per-task timing (used by run_all).
    // Skips if !t.run; otherwise times t.runFn, accumulates runs/totalTimeUs,
    // and disarms (run := false) if run_once && !run_always.
    void _runStep(Task& t, uint32_t now);
};
