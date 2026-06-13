#include "LoopTickOnce.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "RobotState.h"
#include "HaltController.h"
#include "MotionController.h"
#include "MotionCommand.h"
#include <cstdio>

// ---------------------------------------------------------------------------
// loopTickOnce — one iteration of the firmware cooperative loop body.
//
// Called by:
//   - LoopScheduler::run_blocks()  (firmware, after controlCollectSplitPhase)
//   - sim_tick()                   (sim, after hal.tick() + controlCollectSplitPhase)
//
// See LoopTickOnce.h for the full contract.
// ---------------------------------------------------------------------------
void loopTickOnce(Robot& robot, CommandProcessor& cmd, CommandQueue& queue,
                  LoopTickState& ts, uint32_t now)
{
    const RobotConfig& cfg = robot.config;

    // ===== QUEUE: dispatch one enqueued command per tick ===================
    // Commands arrive via cmd.process() → queue.push_back().
    // dequeueOne() dispatches the front command, keeping behaviour identical
    // to the former immediate-dispatch path (enqueue + dequeue in same tick).
    cmd.dequeueOne(queue);

    // ===== SYSTEM WATCHDOG: fire safety_stop + X after sTimeoutMs of silence =
    // ts.watchdogMs == 0 means no command has been received yet this session;
    // the watchdog stays disarmed until the first command arrives.
    // Signed delta avoids uint32 underflow (project memory: watchdog-uint32-underflow).
    //
    // TIME-stop exemption (sprint 024-003): self-terminating commands that
    // carry a TIME stop condition (T, D, G, TURN, RT, G PRE_ROTATE) are
    // exempt from the keepalive requirement — their TIME net fires regardless
    // of host silence.  Open-ended streaming commands (S / VW / R) have no
    // TIME stop and remain keepalive-bound.
    {
        MotionController& mc = robot.motionController;
        bool needsWatchdog =
            (mc.mode() != DriveMode::IDLE) || mc.hasActiveCommand();

        // Exempt commands that carry their own TIME backstop.
        if (mc.hasActiveCommand() && mc.activeCmd().hasTimeStop()) {
            needsWatchdog = false;
        }

        if (cfg.safetyEnabled && ts.watchdogMs != 0 &&
            ts.activeFn != nullptr && needsWatchdog) {
            int32_t wdDelta = (int32_t)(now - ts.watchdogMs);
            if (wdDelta > (int32_t)cfg.sTimeoutMs) {
                ts.watchdogMs = now;  // re-arm to avoid firing every tick
                char wdBuf[64];
                CommandProcessor::replyEvt(wdBuf, sizeof(wdBuf),
                                           "safety_stop", "",
                                           ts.activeFn, ts.activeCtx);
                // Bypass the queue for internal emergency stop: detach queue
                // so process() dispatches X immediately, then restore.
                cmd.setQueue(nullptr);
                cmd.process("X", ts.activeFn, ts.activeCtx);
                cmd.setQueue(&queue);
            }
        }
    }

    // ===== HALT CONDITIONS: evaluate user-registered stop conditions ========
    // Runs after the watchdog check, before the motion tick.
    {
        if (ts.activeFn != nullptr) {
            HaltAction ha = robot.haltController.evaluate(
                robot.state.inputs, now, ts.activeFn, ts.activeCtx);
            // Bypass the queue for halt-triggered emergency stops.
            if (ha == HaltAction::HARD) {
                cmd.setQueue(nullptr);
                cmd.process("X", ts.activeFn, ts.activeCtx);
                cmd.setQueue(&queue);
            } else if (ha == HaltAction::SOFT) {
                cmd.setQueue(nullptr);
                cmd.process("X soft", ts.activeFn, ts.activeCtx);
                cmd.setQueue(&queue);
            }
        }
    }

    // ===== DRIVE: advance drive state machine =================================
    robot.motionController.driveAdvance(
        robot.state.inputs, robot.state.commands, robot.state.target, now);

    // ===== ODOMETRY: dead-reckon pose from encoder deltas ====================
    robot.odometry.predict(robot.state.inputs, cfg.trackwidthMm,
                           cfg.rotationalSlip, now);

    // ===== HAL ACTUATOR TICK: deliver commanded velocity to the HAL ===========
    // Pass the commanded actuator state to the HAL so a bench-mode sensor plant
    // (BenchOtosSensor in NezhaHAL) can integrate it.  Must run before the OTOS
    // block so the plant advances its accumulators before otosCorrect() calls
    // readTransformed().  Production NezhaHAL / MockHAL implement this as a
    // near-no-op when bench mode is off; the robot core no longer reaches into
    // the concrete HAL to do this (034-002, replaces robot.benchOtosTick).
    robot.hal.tick(now, robot.state.commands);

    // ===== OTOS: timed I2C pose read + EKF fusion ============================
    // In firmware: run when enOtos is set and lagOtosMs has elapsed.
    // In sim: the lagOtosMs gate is bypassed; fusion runs every tick when
    // fuseOtos is true (matches the original sim_tick() behaviour which called
    // otosCorrect() unconditionally each tick).
    if (ts.fuseOtos) {
        robot.otosCorrect(now);
    } else if (cfg.lagOtosMs > 0 &&
               (int32_t)(now - ts.lastOtos) >= (int32_t)cfg.lagOtosMs) {
        robot.otosCorrect(now);
        ts.lastOtos = now;
    }

    // ===== LINE: timed I2C read ===============================================
    if (cfg.lagLineMs > 0 &&
        (int32_t)(now - ts.lastLine) >= (int32_t)cfg.lagLineMs) {
        robot.lineRead();
        ts.lastLine = now;
    }

    // ===== COLOUR: timed read =================================================
    if (cfg.lagColorMs > 0 &&
        (int32_t)(now - ts.lastColor) >= (int32_t)cfg.lagColorMs) {
        robot.colorRead();
        ts.lastColor = now;
    }

    // ===== PORTS: timed GPIO read =============================================
    if (cfg.lagPortsMs > 0 &&
        (int32_t)(now - ts.lastPorts) >= (int32_t)cfg.lagPortsMs) {
        robot.portsRead();
        ts.lastPorts = now;
    }

    // ===== TELEMETRY: timed TLM frame emit ====================================
    // N3 fix (030-003): emit with the STREAM-bound fn+ctx pair, not ts.activeCtx
    // (which is the last *command* channel, not the bound stream channel).
    // Mixed serial+radio field setup: STREAM over serial then a radio command
    // would have passed ts.activeCtx = &radio to serialReplyTlm, casting Radio*
    // to SerialPort* — UB.  Using the bound pair keeps TLM on the channel that
    // issued STREAM regardless of which channel subsequent commands arrive on.
    // telemetryEmit guards fn == nullptr, so SET tlmPeriod without STREAM is safe.
    if (cfg.tlmPeriodMs > 0 &&
        (int32_t)(now - ts.lastTlm) >= (int32_t)cfg.tlmPeriodMs) {
        robot.telemetryEmit(now, robot._tlmBoundFn, robot._tlmBoundCtx);
        ts.lastTlm = now;
    }
}
