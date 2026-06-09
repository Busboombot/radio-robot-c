#pragma once
#include "MicroBit.h"
#include "Protocol.h"

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

private:
    Robot&            _robot;
    CommandProcessor& _cmd;
    Communicator&     _comm;
    MicroBit&         _uBit;
};
