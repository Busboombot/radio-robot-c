#pragma once
#include "Protocol.h"     // ReplyFn
#include "CommandQueue.h" // CommandQueue

// Forward declarations — avoids pulling in Robot.h / CommandProcessor.h
// from this header.
struct Robot;
class CommandProcessor;

// ---------------------------------------------------------------------------
// LoopTickState — mutable per-tick state shared between firmware (LoopScheduler)
// and the simulator (SimHandle).
//
// LoopScheduler stores this as _ts; the firmware run_blocks() loop updates
// the active-reply fields (activeFn/Ctx) via runCommsIn() and keeps the
// last* timestamps up-to-date.
//
// SimHandle stores its own copy.  sim_command() arms watchdogMs and sets
// activeFn/Ctx; sim_tick() calls loopTickOnce() with the SimHandle's _ts.
// ---------------------------------------------------------------------------
struct LoopTickState {
    // System keepalive watchdog timestamp (ms).
    // 0 = disarmed (no command received yet this session).
    uint32_t watchdogMs  = 0;

    // Per-timed-block last-run timestamps.
    uint32_t lastOtos    = 0;
    uint32_t lastLine    = 0;
    uint32_t lastColor   = 0;
    uint32_t lastPorts   = 0;
    uint32_t lastTlm     = 0;

    // Active reply sink — updated by runCommsIn() on each inbound command so
    // that async telemetry and EVT completions go back on the originating channel.
    // In the sim, set to storeReply / &replyStore before each sim_tick().
    ReplyFn  activeFn    = nullptr;
    ReplyFn  activeTlmFn = nullptr;
    void*    activeCtx   = nullptr;

    // OTOS EKF fusion.  Firmware always runs the OTOS block when enOtos is set;
    // the sim defaults to false (encoder-only) and enables via sim_set_otos_fusion().
    bool     fuseOtos    = false;
};

// ---------------------------------------------------------------------------
// loopTickOnce — one iteration of the firmware cooperative loop body.
//
// Contains (in order): dequeueOne, watchdog check, halt evaluation,
// driveAdvance, odometry predict, and the timed OTOS/line/colour/ports/TLM
// blocks.
//
// Does NOT include:
//   - controlCollectSplitPhase (caller responsibility — requires hardware clock)
//   - runCommsIn (firmware-only, uses Communicator / SerialPort / Radio)
//   - idle sleep (run_blocks() responsibility)
//
// Parameters:
//   robot   — Robot instance that owns all subsystems.
//   cmd     — CommandProcessor used for emergency stop dispatches (X).
//   queue   — CommandQueue drained by dequeueOne each tick.
//   ts      — mutable tick state (watchdog, timestamps, reply sink, fuseOtos).
//   now     — caller-supplied current time (ms).  Firmware passes systemTime();
//             sim passes g_sim_now_ms.  loopTickOnce() does NOT call systemTime.
// ---------------------------------------------------------------------------
void loopTickOnce(Robot& robot, CommandProcessor& cmd, CommandQueue& queue,
                  LoopTickState& ts, uint32_t now);
