#include "LoopScheduler.h"
#include "LoopTickOnce.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "Communicator.h"
#include "SerialPort.h"
#include "Radio.h"
#include "RobotState.h"
#include "HaltController.h"
#include <cstdio>

// ---------------------------------------------------------------------------
// Reply-sink adapters.
//
// serialReply    — used for command replies + EVT completions (reliable,
//                  bounded-wait send that won't silently drop).
// serialReplyTlm — used for the telemetry stream (pure async, drop-tolerant;
//                  never stalls the loop for a momentarily-full TX buffer).
// radioReply     — used when a command arrived over radio.
// ---------------------------------------------------------------------------

static void serialReply(const char* msg, void* ctx)
{
    static_cast<SerialPort*>(ctx)->sendReliable(msg);
}

static void serialReplyTlm(const char* msg, void* ctx)
{
    static_cast<SerialPort*>(ctx)->send(msg);
}

static void radioReply(const char* msg, void* ctx)
{
    static_cast<Radio*>(ctx)->send(msg);
}

// ---------------------------------------------------------------------------
// runCommsIn — drain serial and radio command queues each iteration.
//
// activeFn / activeCtx are updated to the originating channel so that async
// EVT completions go back on the channel that sent the command.
//
// D10 channel binding (028-005): activeTlmFn is NO LONGER updated per-command.
// Instead it is read from robot._tlmBoundFn — the channel that last issued a
// STREAM command.  This prevents a radio command from silently redirecting
// the serial TLM stream.  If no STREAM has been issued, _tlmBoundFn is
// nullptr and TLM is suppressed (same as tlmPeriodMs=0 on init).
// ---------------------------------------------------------------------------

static void runCommsIn(LoopScheduler& sched, uint32_t now)
{
    CommandProcessor& cmd    = sched.cmd();
    SerialPort&       serial = sched.comm().serial();
    Radio&            radio  = sched.comm().radio();

    char buf[512];

    while (serial.readLine(buf, sizeof(buf))) {
        sched.activeFn  = serialReply;
        sched.activeCtx = &serial;
        cmd.process(buf, serialReply, &serial);
        sched.resetWatchdog(now);
    }

    while (radio.poll(buf, sizeof(buf))) {
        sched.activeFn  = radioReply;
        sched.activeCtx = &radio;
        cmd.process(buf, radioReply, &radio);
        sched.resetWatchdog(now);
    }

    // D10 channel binding (028-005): derive the TLM fn from the bound ctx.
    // handleStream stores robot._tlmBoundCtx = replyCtx (the channel's ctx ptr).
    // By comparing against &serial and &radio we select the TLM-appropriate fn:
    //   serial → serialReplyTlm  (async, drop-tolerant — never stalls the loop)
    //   radio  → radioReply      (no async variant needed for radio)
    //   other  → nullptr         (unbound; suppresses TLM until STREAM issued)
    // This is the ONLY place activeTlmFn is updated, so commands on other
    // channels do not redirect the TLM stream.
    void* bCtx = sched.robot()._tlmBoundCtx;
    if (bCtx == static_cast<void*>(&serial)) {
        sched.robot()._tlmBoundFn = serialReplyTlm;
    } else if (bCtx == static_cast<void*>(&radio)) {
        sched.robot()._tlmBoundFn = radioReply;
    }
    // If bCtx is nullptr (never bound) or some other ctx (sim), leave
    // _tlmBoundFn unchanged.
    sched.activeTlmFn = sched.robot()._tlmBoundFn;
}

// ---------------------------------------------------------------------------
// LoopScheduler constructor
// ---------------------------------------------------------------------------

LoopScheduler::LoopScheduler(Robot& robot, CommandProcessor& cmd,
                             Communicator& comm, MicroBit& uBit)
    : activeFn(_ts.activeFn),
      activeTlmFn(_ts.activeTlmFn),
      activeCtx(_ts.activeCtx),
      _robot(robot),
      _cmd(cmd),
      _comm(comm),
      _uBit(uBit)
{
    // Wire the queue: commands arriving via process() are enqueued; the tick
    // body drains one per iteration via dequeueOne(), keeping behaviour
    // transparent in run_blocks() mode (enqueue + dequeue in same tick).
    _cmd.setQueue(&_queue);

    // Wire the same queue into Robot's MotionCtx so VW converter handlers
    // (S, T, D, G, R, TURN) can push_front a VW ParsedCommand.
    // Sprint 026-002: replaced robot.motionController.setQueue() with
    // robot.setMotionQueue() since MotionCtx (and its queue pointer) now
    // lives in Robot, not MotionController.
    _robot.setMotionQueue(&_queue);
}

// ---------------------------------------------------------------------------
// run_test — serial-only hardware-free dispatch loop. Never returns.
//
// Reads from serial only (no radio). Commands arriving via process() are
// enqueued via _queue. The inner drain dispatches non-hardware commands
// normally; commands flagged CMD_ACCESS_HARDWARE are discarded with a
// "DBG skip <prefix>" notice on serial.
//
// This lets the full command-transformation chain be exercised without
// running motors: S/T/D/G/R/TURN handlers execute and push VW to the
// queue; the VW entry is then detected as CMD_ACCESS_HARDWARE and skipped.
//
// Swap sched.run_blocks() → sched.run_test() in main.cpp for a test build.
// ---------------------------------------------------------------------------

void LoopScheduler::run_test()
{
    // Re-wire the queue: main.cpp Phase 3 reassigns CommandProcessor via
    // operator=, which resets _queue to nullptr. Re-set it here so that
    // process() enqueues rather than dispatching immediately, enabling the
    // hardware-flag filter below.
    _cmd.setQueue(&_queue);

    SerialPort& serial = _comm.serial();
    char buf[512];

    while (true) {
        // 1. Drain serial input (no radio — hardware-free loop).
        while (serial.readLine(buf, sizeof(buf))) {
            _cmd.process(buf, serialReply, &serial);
            resetWatchdog(_uBit.systemTime());
        }

        // 2. Drain queue, filtering hardware commands.
        ParsedCommand pc;
        while (_queue.pop_front(pc)) {
            if (pc.desc && (pc.desc->flags & CMD_ACCESS_HARDWARE)) {
                char skip[64];
                snprintf(skip, sizeof(skip), "DBG skip %s\n", pc.desc->prefix);
                serialReply(skip, &serial);
            } else if (pc.desc && pc.desc->handlerFn) {
                pc.desc->handlerFn(pc.args, pc.corrId,
                                   pc.replyFn, pc.replyCtx,
                                   pc.desc->handlerCtx);
            }
        }

        _uBit.sleep(10);
    }
}

// ---------------------------------------------------------------------------
// run_blocks — the cooperative main loop. Never returns.
//
// Each iteration:
//   1. CONTROL: read both encoders → velocity PID → write PWM (every tick).
//   2. Comms drain: read serial + radio queues (runCommsIn).
//   3. Tick body: loopTickOnce() — dequeue, watchdog, halt, drive, odometry,
//      OTOS, line, colour, ports, TLM.
//   4. IDLE SLEEP until the control deadline.
//
// Time checks use signed deltas — (int32_t)(now - last) — to survive uint32
// millisecond wrap without the underflow that caused the watchdog notch bug.
//
// Enable flags (en*) let you turn a block off without touching the timing.
// Periods are read from cfg each iteration so SET lag.* takes effect live.
// ---------------------------------------------------------------------------

void LoopScheduler::run_blocks()
{
    // ---- ENABLE FLAGS -------------------------------------------------------
    bool enControl = true;   // read encoders + run PID + write motors (metronome)
    bool enComms   = true;   // drain serial + radio command queues
    // Note: drive, odometry, and sensor blocks are enabled unconditionally inside
    // loopTickOnce().  The former enDrive/enOdom/enOtos/etc. local booleans are
    // no longer needed here.

    // ---- Per-block last-run timestamps for timed blocks ---------------------
    // Seeded to NOW (not 0) so each block waits a full period before first run,
    // preventing a burst of I2C activity on tick 1. Small descending offsets
    // phase-spread the blocks so they don't keep firing together.
    uint32_t t0 = _uBit.systemTime();
    _ts.lastOtos  = t0;
    _ts.lastLine  = t0 - 10;
    _ts.lastColor = t0 - 20;
    _ts.lastPorts = t0 - 30;
    _ts.lastTlm   = t0 - 40;

    uint32_t controlDeadline = 0;

    while (true) {
        uint32_t now = _uBit.systemTime();

        // ===== CONTROL: read both encoders (M1 first) → PID → setSpeed =====
        if (enControl) {
            _robot.controlCollectSplitPhase(now, 0);
        }
        now = _uBit.systemTime();
        controlDeadline = now + (uint32_t)_robot.config.controlPeriodMs;

        // ===== COMMS: drain serial + radio (every iteration) ================
        if (enComms) {
            now = _uBit.systemTime();
            runCommsIn(*this, now);
        }

        // ===== TICK BODY: dequeue, watchdog, halt, drive, odometry, sensors =
        now = _uBit.systemTime();
        loopTickOnce(_robot, _cmd, _queue, _ts, now);

        // ===== IDLE SLEEP until the control deadline ==========================
        now = _uBit.systemTime();
        if ((int32_t)(controlDeadline - now) > 0) {
            _uBit.sleep(controlDeadline - now);
        }
    }
}
