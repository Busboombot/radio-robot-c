#pragma once
#include "MicroBit.h"
#include "LoopTickOnce.h"  // LoopTickState + loopTickOnce

// Forward declarations to avoid pulling in the full header graph.
struct Robot;
class CommandProcessor;
class Communicator;

// ---------------------------------------------------------------------------
// LoopScheduler — single cooperative main loop for the robot firmware.
//
// Runs run_blocks() — a straightforward fully-inlined loop. Every subsystem
// is an explicit block in the loop body, each gated by a plain on/off enable
// flag and a signed-delta time check (avoids uint32 subtraction underflow).
//
// The idle sleep at the bottom of each iteration paces the loop to a fixed
// controlPeriodMs deadline.
//
// Construction:
//   LoopScheduler sched(robot, cmd, comm, uBit);
//   sched.run_blocks();   // never returns
// ---------------------------------------------------------------------------
class LoopScheduler {
public:
    LoopScheduler(Robot& robot, CommandProcessor& cmd, Communicator& comm, MicroBit& uBit);

    // The main cooperative loop. Never returns.
    void run_blocks();

    // Serial-only hardware-free test loop. Never returns.
    // Drains serial input only (no radio); dispatches non-hardware commands;
    // reports "DBG skip <prefix>" for CMD_ACCESS_HARDWARE commands.
    // Swap into main.cpp in place of run_blocks() to run a test build.
    void run_test();

    // ---------------------------------------------------------------------------
    // Accessors used by task functions and command handlers.
    // ---------------------------------------------------------------------------
    Robot&            robot() { return _robot; }
    CommandProcessor& cmd()   { return _cmd;   }
    Communicator&     comm()  { return _comm;  }
    MicroBit&         uBit()  { return _uBit;  }

    // Active reply sink — updated each time a command is dispatched so that
    // telemetry and event completions go back over the originating channel.
    //
    // These are reference members that alias _ts.activeFn / _ts.activeTlmFn /
    // _ts.activeCtx, so callers (main.cpp, MotorController) that store
    // &sched.activeFn / &sched.activeCtx still point at the live field.
    ReplyFn& activeFn;       // command replies + EVT (reliable send)
    ReplyFn& activeTlmFn;    // telemetry stream (ASYNC, drop-tolerant)
    void*&   activeCtx;

    // Reset the system keepalive watchdog timestamp.
    // Called by runCommsIn() after each inbound command is dispatched,
    // and by Robot.cpp's DBG watchdog-reset handler.
    void resetWatchdog(uint32_t now_ms) { _ts.watchdogMs = now_ms; }

private:
    Robot&            _robot;
    CommandProcessor& _cmd;
    Communicator&     _comm;
    MicroBit&         _uBit;

    // Per-tick mutable state: watchdog, last-run timestamps, active reply sink.
    // run_blocks() initialises the last* timestamps to the current time before
    // entering the loop so that each timed block waits a full period on first run.
    // loopTickOnce() reads and updates these fields each tick.
    LoopTickState     _ts;

    // Command queue — owned by LoopScheduler, wired to both cmd and
    // motionController at construction so converter handlers can push_front VW.
    CommandQueue      _queue;
};
