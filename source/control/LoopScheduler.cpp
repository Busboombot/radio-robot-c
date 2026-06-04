#include "LoopScheduler.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "SerialPort.h"
#include "Radio.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "PortIO.h"
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
    Robot&            robot  = sched.robot();
    CommandProcessor& cmd    = sched.cmd();
    SerialPort&       serial = robot.serialPort();
    Radio&            radio  = robot.radioPort();

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

// line-read: read the 4-channel line sensor into HardwareState::line[].
static void runLineRead(LoopScheduler& sched, uint32_t now)
{
    LineSensor* ls = sched.robot().lineSensor();
    if (!ls) return;

    HardwareState& inputs = sched.robot().stateMut().inputs;
    if (ls->readValues(inputs.line)) {
        inputs.lineVS.lastUpdMs = now;
        inputs.lineVS.valid     = true;
    }
}

// color-read: non-blocking poll of the RGBC color sensor.
static void runColorRead(LoopScheduler& sched, uint32_t now)
{
    ColorSensor* cs = sched.robot().colorSensor();
    if (!cs) return;

    HardwareState& inputs = sched.robot().stateMut().inputs;
    if (cs->pollRGBC(inputs.colorR, inputs.colorG, inputs.colorB, inputs.colorC)) {
        inputs.colorVS.lastUpdMs = now;
        inputs.colorVS.valid     = true;
    }
}

// ports-read: read digital and analogue GPIO ports.
static void runPortsRead(LoopScheduler& sched, uint32_t now)
{
    PortIO&        ports  = sched.robot().portIO();
    HardwareState& inputs = sched.robot().stateMut().inputs;

    for (uint8_t i = 0; i < 4; ++i) {
        inputs.digitalIn[i] = (ports.readDigital(i) != 0);
        inputs.analogIn[i]  = (int16_t)ports.readAnalog(i);
    }
    inputs.portsVS.lastUpdMs = now;
    inputs.portsVS.valid     = true;
}

// telemetry-emit: assemble and send the unified TLM frame.
static void runTelemetryEmit(LoopScheduler& sched, uint32_t now)
{
    sched.robot().telemetryTick(now, sched.activeFn, sched.activeCtx);
}

// ---------------------------------------------------------------------------
// LoopScheduler constructor
//
// Initialises the task table with name / periodMs / lastRunMs / estCostMs /
// due / run for each of the 8 low-priority tasks.
// Period values are seeded from the robot config at construction time; they
// can be changed at runtime by the command processor (SET lag.*).
// ---------------------------------------------------------------------------

LoopScheduler::LoopScheduler(Robot& robot, CommandProcessor& cmd, MicroBit& uBit)
    : activeFn(nullptr),
      activeCtx(nullptr),
      _robot(robot),
      _cmd(cmd),
      _uBit(uBit),
      _cursor(0),
      _pendingWheel(0),
      _controlDeadline(0)
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
// controlFireRequest — private.
//
// Fires the encoder request for the OTHER wheel (alternates L/R).
// Updates _pendingWheel for the next iteration's collect phase.
//
// This is called LAST before the idle sleep, keeping the motor's
// pending-read window free of competing I2C from sensor tasks.
//
// Alternation:
//   _pendingWheel == 0 (first): fire left → set _pendingWheel = 1
//   _pendingWheel == 1 (left):  fire right → set _pendingWheel = 2
//   _pendingWheel == 2 (right): fire left  → set _pendingWheel = 1
// ---------------------------------------------------------------------------

void LoopScheduler::controlFireRequest()
{
    // Alternation:
    //   pendingWheel == 0 (first iter): fire left  → pendingWheel = 1
    //   pendingWheel == 1 (left just collected):  fire right → pendingWheel = 2
    //   pendingWheel == 2 (right just collected): fire left  → pendingWheel = 1
    if (_pendingWheel == 1) {
        // Left was just collected; fire right next.
        _robot.controlFireRequest(2);
        _pendingWheel = 2;
    } else {
        // Either first iteration (0) or right was just collected (2): fire left.
        _robot.controlFireRequest(1);
        _pendingWheel = 1;
    }
}

// ---------------------------------------------------------------------------
// run — the cooperative main loop. Never returns.
//
// Iteration structure:
//
//   1. HARD TASK (always first):
//      a. Collect the pending encoder (split-phase collect).
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
//   3. ENCODER REQUEST (last I2C op before idle):
//      controlFireRequest() fires the encoder request for the alternate wheel.
//      Ordering rule: this is always AFTER all sensor-I2C tasks in the sweep.
//
//   4. IDLE SLEEP:
//      uBit.sleep(controlDeadline - now) — the program's only sleep.
//      The sleep provides the required ≥ one-loop-period delay between
//      requestEncoder() and the collectEncoder() at the top of the next iteration.
// ---------------------------------------------------------------------------

void LoopScheduler::run()
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
        // Sync telemetry-emit period from config each iteration (the STREAM
        // command can change tlmPeriodMs at runtime).
        _table[7].periodMs = (uint32_t)_robot.config().tlmPeriodMs;

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
            t.run(*this, now);
            t.lastRunMs = now;

            // Post-task deadline re-check: bail if control is due.
            if (_uBit.systemTime() >= _controlDeadline) {
                break;
            }
        }

        // ------------------------------------------------------------------
        // 3. ENCODER REQUEST — LAST I2C op before idle.
        //    Fires for the alternate wheel; the idle sleep below provides
        //    the ≥ one-loop-period delay before the next collect.
        // ------------------------------------------------------------------
        controlFireRequest();

        // ------------------------------------------------------------------
        // 4. IDLE SLEEP until the control deadline.
        // ------------------------------------------------------------------
        now = _uBit.systemTime();
        if (now < _controlDeadline) {
            _uBit.sleep(_controlDeadline - now);
        }
    }
}
