#pragma once
#include "MicroBit.h"
#include "Protocol.h"
#include "CommandQueue.h"

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

    // ---------------------------------------------------------------------------
    // Accessors used by task functions and command handlers.
    // ---------------------------------------------------------------------------
    Robot&            robot() { return _robot; }
    CommandProcessor& cmd()   { return _cmd;   }
    Communicator&     comm()  { return _comm;  }
    MicroBit&         uBit()  { return _uBit;  }

    // Active reply sink — updated each time a command is dispatched so that
    // telemetry and event completions go back over the originating channel.
    ReplyFn  activeFn;       // command replies + EVT (reliable send)
    ReplyFn  activeTlmFn;    // telemetry stream (ASYNC, drop-tolerant)
    void*    activeCtx;

    // Reset the system keepalive watchdog timestamp.
    // Called by runCommsIn() after each inbound command is dispatched.
    void resetWatchdog(uint32_t now_ms) { _watchdogMs = now_ms; }

private:
    Robot&            _robot;
    CommandProcessor& _cmd;
    Communicator&     _comm;
    MicroBit&         _uBit;

    // System keepalive watchdog (Sprint 020, Ticket 005).
    // Reset in runCommsIn() on every inbound command.
    // Fires EVT safety_stop + X if sTimeoutMs passes without any inbound command.
    // 0 = not yet armed (no command received yet this session).
    uint32_t _watchdogMs = 0;

    // Command queue — owned by LoopScheduler, set on CommandProcessor at boot.
    // Commands arriving via runCommsIn() are enqueued; the tick body drains one
    // per iteration via cmd.dequeueOne(_queue), keeping behaviour transparent.
    CommandQueue _queue;
};
