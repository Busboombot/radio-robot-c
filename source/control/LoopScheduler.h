#pragma once
#include "MicroBit.h"
#include "Protocol.h"

// Forward declarations to avoid pulling in the full header graph.
class Robot;
class CommandProcessor;

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
//   run        : executes the task body.
// ---------------------------------------------------------------------------
struct Task {
    const char* name;
    uint32_t    periodMs;
    uint32_t    lastRunMs;
    uint32_t    estCostMs;
    bool      (*due)(struct Task& task, uint32_t now);
    void      (*run)(class LoopScheduler& sched, uint32_t now);
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
//   LoopScheduler sched(robot, cmd, uBit);
//   sched.run();   // never returns
// ---------------------------------------------------------------------------
class LoopScheduler {
public:
    LoopScheduler(Robot& robot, CommandProcessor& cmd, MicroBit& uBit);

    // Enter the cooperative main loop. Never returns.
    void run();

    // ---------------------------------------------------------------------------
    // Accessors used by Task::run() lambdas.
    // ---------------------------------------------------------------------------
    Robot&            robot() { return _robot; }
    CommandProcessor& cmd()   { return _cmd;   }
    MicroBit&         uBit()  { return _uBit;  }

    // Active reply sink — updated each time a command is dispatched so that
    // telemetry and event completions go back over the originating channel.
    ReplyFn  activeFn;
    void*    activeCtx;

private:
    Robot&            _robot;
    CommandProcessor& _cmd;
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
};
