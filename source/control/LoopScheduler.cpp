#include "LoopScheduler.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "Communicator.h"
#include "SerialPort.h"
#include "Radio.h"
#include "RobotState.h"
#include "HaltController.h"

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
// Updates activeFn/activeTlmFn/activeCtx on the scheduler so that async
// telemetry and EVT completions go back on the channel that sent the command.
// ---------------------------------------------------------------------------

static void runCommsIn(LoopScheduler& sched, uint32_t now)
{
    CommandProcessor& cmd    = sched.cmd();
    SerialPort&       serial = sched.comm().serial();
    Radio&            radio  = sched.comm().radio();

    char buf[512];

    while (serial.readLine(buf, sizeof(buf))) {
        sched.activeFn    = serialReply;
        sched.activeTlmFn = serialReplyTlm;
        sched.activeCtx   = &serial;
        cmd.process(buf, serialReply, &serial);
        sched.resetWatchdog(now);
    }

    while (radio.poll(buf, sizeof(buf))) {
        sched.activeFn    = radioReply;
        sched.activeTlmFn = radioReply;
        sched.activeCtx   = &radio;
        cmd.process(buf, radioReply, &radio);
        sched.resetWatchdog(now);
    }
}

// ---------------------------------------------------------------------------
// LoopScheduler constructor
// ---------------------------------------------------------------------------

LoopScheduler::LoopScheduler(Robot& robot, CommandProcessor& cmd,
                             Communicator& comm, MicroBit& uBit)
    : activeFn(nullptr),
      activeTlmFn(nullptr),
      activeCtx(nullptr),
      _robot(robot),
      _cmd(cmd),
      _comm(comm),
      _uBit(uBit)
{
    // Wire the queue: commands arriving via process() are enqueued; the tick
    // body drains one per iteration via dequeueOne(), keeping behaviour
    // transparent in run_blocks() mode (enqueue + dequeue in same tick).
    _cmd.setQueue(&_queue);
}

// ---------------------------------------------------------------------------
// run_blocks — the cooperative main loop. Never returns.
//
// Each iteration:
//   1. CONTROL: read both encoders → velocity PID → write PWM (every tick).
//   2. Timed blocks: comms, drive, odometry, OTOS, line, colour, ports, TLM.
//   3. IDLE SLEEP until the control deadline.
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
    bool enDrive   = true;   // advance S/T/D/G drive state machine
    bool enOdom    = true;   // dead-reckon pose from the encoders
    bool enOtos    = true;   // OTOS pose read + complementary fusion (timed)
    bool enLine    = true;   // line sensor read (timed)
    bool enColor   = true;   // colour sensor read (timed)
    bool enPorts   = true;   // port-IO / GPIO read (timed)
    bool enTlm     = true;   // assemble + send the TLM telemetry frame (timed)

    // ---- Per-block last-run timestamps for timed blocks ---------------------
    // Seeded to NOW (not 0) so each block waits a full period before first run,
    // preventing a burst of I2C activity on tick 1. Small descending offsets
    // phase-spread the blocks so they don't keep firing together.
    uint32_t t0 = _uBit.systemTime();
    uint32_t lastOtos  = t0;
    uint32_t lastLine  = t0 - 10;
    uint32_t lastColor = t0 - 20;
    uint32_t lastPorts = t0 - 30;
    uint32_t lastTlm   = t0 - 40;

    uint32_t controlDeadline = 0;

    while (true) {
        const RobotConfig& cfg = _robot.config;
        uint32_t now = _uBit.systemTime();

        // ===== CONTROL: read both encoders (M1 first) → PID → setSpeed =====
        if (enControl) {
            _robot.controlCollectSplitPhase(now, 0);
        }
        now = _uBit.systemTime();
        controlDeadline = now + (uint32_t)cfg.controlPeriodMs;

        // ===== COMMS: drain serial + radio (every iteration) ================
        if (enComms) {
            now = _uBit.systemTime();
            runCommsIn(*this, now);
        }

        // ===== QUEUE: dispatch one enqueued command per tick =================
        // runCommsIn() enqueues commands via cmd.process() → _queue.push_back().
        // dequeueOne() dispatches the front command, keeping behaviour identical
        // to the former immediate-dispatch path (enqueue + dequeue in same tick).
        _cmd.dequeueOne(_queue);

        // ===== SYSTEM WATCHDOG: fire safety_stop + X after sTimeoutMs of silence =====
        // _watchdogMs == 0 means no command has been received yet this session;
        // the watchdog stays disarmed until the first command arrives.
        // Signed delta avoids uint32 underflow (see memory note: watchdog-uint32-underflow).
        //
        // Only fires when the robot is in an open-ended motion mode:
        //   - STREAMING (S command, no active MotionCommand)
        //   - VELOCITY/ARC (VW/R, active MotionCommand with no stop conditions)
        // Self-terminating commands (T/D/G/TURN, with stop conditions) manage
        // their own lifetime and are NOT interrupted by the system watchdog.
        {
            now = _uBit.systemTime();
            MotionController& mc = _robot.motionController;
            bool needsWatchdog =
                (mc.mode() == DriveMode::STREAMING) ||
                (mc.hasActiveCommand() && mc.activeCmd().isOpenEnded());
            if (_watchdogMs != 0 && activeFn != nullptr && needsWatchdog) {
                int32_t wdDelta = (int32_t)(now - _watchdogMs);
                if (wdDelta > (int32_t)_robot.config.sTimeoutMs) {
                    _watchdogMs = now;  // reset to avoid repeated firing every tick
                    char wdBuf[64];
                    CommandProcessor::replyEvt(wdBuf, sizeof(wdBuf),
                                               "safety_stop", "",
                                               activeFn, activeCtx);
                    // Bypass the queue for internal emergency stop: set queue to
                    // null so process() dispatches X immediately, then restore.
                    _cmd.setQueue(nullptr);
                    _cmd.process("X", activeFn, activeCtx);
                    _cmd.setQueue(&_queue);
                }
            }
        }

        // ===== HALT CONDITIONS: evaluate user-registered stop conditions =====
        // Runs after the watchdog check, before the motion tick.
        // When a condition fires: emit EVT halt id=<n>, dispatch X or X soft.
        {
            now = _uBit.systemTime();
            if (activeFn != nullptr) {
                HaltAction ha = _robot.haltController.evaluate(
                    _robot.state.inputs, now, activeFn, activeCtx);
                // Bypass the queue for halt-triggered emergency stops: detach
                // queue so process() dispatches immediately, then restore.
                if (ha == HaltAction::HARD) {
                    _cmd.setQueue(nullptr);
                    _cmd.process("X", activeFn, activeCtx);
                    _cmd.setQueue(&_queue);
                } else if (ha == HaltAction::SOFT) {
                    _cmd.setQueue(nullptr);
                    _cmd.process("X soft", activeFn, activeCtx);
                    _cmd.setQueue(&_queue);
                }
            }
        }

        // ===== DRIVE: advance drive state machine ==========================
        if (enDrive) {
            now = _uBit.systemTime();
            _robot.motionController.driveAdvance(
                _robot.state.inputs, _robot.state.commands, _robot.state.target, now);
        }

        // ===== ODOMETRY: dead-reckon pose from encoder deltas ===============
        if (enOdom) {
            _robot.odometry.predict(_robot.state.inputs, _robot.config.trackwidthMm);
        }

        // ===== OTOS: timed I2C pose read + fusion ===========================
        now = _uBit.systemTime();
        if (enOtos && cfg.lagOtosMs > 0 &&
            (int32_t)(now - lastOtos) >= (int32_t)cfg.lagOtosMs) {
            _robot.otosCorrect(now);
            lastOtos = now;
        }

        // ===== LINE: timed I2C read ==========================================
        now = _uBit.systemTime();
        if (enLine && cfg.lagLineMs > 0 &&
            (int32_t)(now - lastLine) >= (int32_t)cfg.lagLineMs) {
            _robot.lineRead();
            lastLine = now;
        }

        // ===== COLOUR: timed read ============================================
        now = _uBit.systemTime();
        if (enColor && cfg.lagColorMs > 0 &&
            (int32_t)(now - lastColor) >= (int32_t)cfg.lagColorMs) {
            _robot.colorRead();
            lastColor = now;
        }

        // ===== PORTS: timed GPIO read =========================================
        now = _uBit.systemTime();
        if (enPorts && cfg.lagPortsMs > 0 &&
            (int32_t)(now - lastPorts) >= (int32_t)cfg.lagPortsMs) {
            _robot.portsRead();
            lastPorts = now;
        }

        // ===== TELEMETRY: timed TLM frame emit ================================
        now = _uBit.systemTime();
        if (enTlm && cfg.tlmPeriodMs > 0 &&
            (int32_t)(now - lastTlm) >= (int32_t)cfg.tlmPeriodMs) {
            _robot.telemetryEmit(now, activeTlmFn, activeCtx);
            lastTlm = now;
        }

        // ===== IDLE SLEEP until the control deadline ==========================
        now = _uBit.systemTime();
        if ((int32_t)(controlDeadline - now) > 0) {
            _uBit.sleep(controlDeadline - now);
        }
    }
}
