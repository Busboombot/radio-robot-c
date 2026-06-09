#pragma once
#include "MicroBit.h"
#include "Motor.h"
#include "Config.h"
#include "RatioPidController.h"
#include "VelocityController.h"
#include "RobotState.h"
#include "Protocol.h"
#include <stdint.h>

// Forward declaration — include in .cpp only.
class I2CBus;

/**
 * MotorController — per-wheel velocity PID wheel speed control.
 *
 * Inner loop is VelocityController (PI+FF) — one instance per wheel.
 * RatioPidController is retained but bypassed (not called in normal drive).
 *
 * Sprint 010 replaces the cumulative-distance ratio PID inner loop with
 * two independent VelocityController instances (_vcL, _vcR) that track
 * per-wheel mm/s setpoints. See docs/kinematics-model.md §2.1.
 *
 * Thread safety: single-threaded tick loop only.
 */
class MotorController {
public:
    MotorController(Motor& left, Motor& right, const RobotConfig& cal);

    // Gains — public so CommandProcessor can update via K-commands.
    // Defaults: kFF=0.15, kP=0.05, kI=0.20, iClamp=60, kRatio=0.01
    struct Gains {
        float kFF;      // feed-forward coefficient
        float kP;       // proportional gain
        float kI;       // integral gain
        float iClamp;   // integral windup clamp (PWM units, ±)
        float kRatio;   // ratio cross-coupling gain (sprint 2 stub, small)
    } gains;

    // Set speed targets in mm/s. Zero both to coast (not brake).
    void setTarget(float leftMms, float rightMms);

    /**
     * startDriveClean — used by T, D, and G commands.
     * Full clean start: snapshot encoders, compute ratio, reset PID.
     * Always call this when starting a new bounded command.
     */
    void startDriveClean(float leftMms, float rightMms);

    /**
     * startDrive — used by the S (streaming) command only.
     * Re-seeds cmdEncStart to preserve accumulated ratio history across keepalive re-sends.
     * Does NOT reset PID unless the faster/slower assignment changes.
     */
    void startDrive(float leftMms, float rightMms);

    // Stop: zero targets, reset PID, and write zero PWM.
    void stop();

    // Reset integrators only (called by CommandProcessor on mode change).
    void resetIntegrators();

    // Update PID gains at runtime (called by K-command setters).
    void updatePidGains(float kP, float kI, float kD, float iClamp);

    // Push per-wheel velocity gains (vel.kP/kI/kFF/iMax/kAw/minWheel) from config
    // into the live VelocityControllers after a SET. (filt/sync are read per-tick.)
    void updateVelGains(const RobotConfig& cal);

    /**
     * controlTick — cooperative-loop control step (014-003 / 014-007).
     *
     * Per-wheel zero-order-hold velocity (014-007 ZOH fix):
     *   refreshedWheel: 0 = none (first iteration / sync fallback),
     *                   1 = left wheel was just collected,
     *                   2 = right wheel was just collected.
     *
     * For the refreshed wheel only: computes velocity as
     *   (inputs.encWMm - _prevEncW) / elapsed_s
     * using per-wheel timestamps for the correct elapsed time, then
     * writes inputs.velWMms and updates _prevEncW / _prevTimeMsW.
     *
     * For the other wheel: leaves inputs.velWMms unchanged (ZOH).
     *
     * Then runs PID for BOTH wheels using the held inputs.velLMms/velRMms,
     * writes cmds.pwmL/R, and calls Motor::setSpeed().
     *
     * now_ms: current system time in milliseconds (for per-wheel dt).
     */
    void controlTick(HardwareState& inputs, MotorCommands& cmds,
                     uint32_t now_ms, int refreshedWheel);

    /**
     * setCommandsRef — bind the authoritative MotorCommands struct so that
     * setTarget / startDrive / stop can write tgtLMms/R directly (014-007).
     * Must be called before the first setTarget (Robot constructor does this).
     */
    void setCommandsRef(MotorCommands* cmds) { _cmds = cmds; }

    /**
     * setI2CBus — bind the I2CBus instance so controlTick() can read per-device
     * stats when emitting EVT enc_wedged (015-003).
     * Optional — if unset, EVT stats show zeros.
     */
    void setI2CBus(I2CBus* bus) { _i2cBus = bus; }

    /**
     * setEvtSink — bind the reply sink for EVT enc_wedged emission (015-003).
     * Points at the LoopScheduler's activeFn / activeCtx live fields so the
     * event goes to whichever channel sent the most recent command.
     * Optional — if fn is nullptr, the event is silently dropped.
     */
    void setEvtSink(ReplyFn* fn, void** ctx) { _evtFn = fn; _evtCtx = ctx; }

    /**
     * stuckCountL / stuckCountR — current consecutive-identical-while-commanded
     * counter for left and right wheels respectively (015-003).
     * Read by CommandProcessor::DBG I2C to include in the dump line.
     */
    uint8_t stuckCountL() const { return _stuckCountL; }
    uint8_t stuckCountR() const { return _stuckCountR; }

    /** resetStuckCounters — zero both stuck counters and re-arm both latch flags. */
    void resetStuckCounters();

    /**
     * getVelocitySourceFlags — always returns false for both wheels.
     *
     * The chip readSpeed (0x47) path was disabled in sprint 013 to fix
     * motor throb. The encoder-delta fallback is the sole velocity source.
     * Method retained for CommandProcessor wire-format compatibility until
     * ticket 007 migrates callers to Robot::state.
     */
    void getVelocitySourceFlags(bool& leftChip, bool& rightChip) const;

    // Read cumulative encoder positions in mm (sum since last resetEncoderAccumulators()).
    void getEncoderPositions(int32_t& leftMm, int32_t& rightMm) const;

    // Zero encoder accumulators — delegates to Motor::resetEncoder() for each wheel.
    void resetEncoderAccumulators();

private:
    Motor&             _motorL;
    Motor&             _motorR;
    const RobotConfig& _cal;

    // Per-wheel velocity controllers (PI + feed-forward). Sprint 010 inner loop.
    VelocityController _vcL;
    VelocityController _vcR;

    // RatioPidController retained for compile compatibility; bypassed in normal drive.
    RatioPidController _pid;

    // Ratio-PID bookkeeping fields — retained but unused in velocity-loop path.
    float _cmdEncStartL;     // encoder mm snapshot at command start (left)
    float _cmdEncStartR;     // encoder mm snapshot at command start (right)
    float _cmdRatio;         // |fasterSpeed| / |slowerSpeed|, always >= 1.0
    bool  _fasterIsRight;    // true if right wheel is the commanded-faster wheel

    // Pointer to the authoritative MotorCommands (set by Robot via setCommandsRef).
    // setTarget / startDrive / stop write tgtLMms/R here directly (014-007).
    MotorCommands* _cmds;

    // Previous encoder snapshots — intermediate compute state for velocity differentiation.
    // These are NOT robot state (they are algorithm state private to MotorController).
    float    _prevEncL;   // mm at start of last controlTick for left wheel
    float    _prevEncR;   // mm at start of last controlTick for right wheel

    // Per-wheel timing for ZOH velocity computation (014-007).
    // lastUpdMs is 0 until the first valid reading (first-sample guard).
    uint32_t _prevTimeMsL;  // system time (ms) of last left-wheel collect
    uint32_t _prevTimeMsR;  // system time (ms) of last right-wheel collect
    bool     _hasTimestampL;
    bool     _hasTimestampR;

    // PID integrator dt: actual elapsed control-tick time (ms). The loop runs at
    // ~24 ms (10 ms nominal + 2x4 ms encoder settle + bus), NOT the nominal
    // controlPeriodMs, so using the nominal value made kI act at ~0.4x strength
    // and never close the steady-state error. Use the measured delta, clamped to
    // a sane window so a stalled tick can't spike the integrator.
    uint32_t _lastPidMs;     // system time (ms) of last controlTick PID update
    bool     _hasPidTick;    // false until the first PID tick

    // -------------------------------------------------------------------------
    // Encoder-wedge detector (015-003)
    //
    // Per-wheel consecutive-identical-while-commanded counter.
    // When either counter reaches kWedgeThreshold and the latch flag is clear,
    // an EVT enc_wedged line is emitted via _evtFn/_evtCtx and the latch is set.
    // The latch re-arms (clears) when the encoder value changes.
    // -------------------------------------------------------------------------
    static constexpr uint8_t kWedgeThreshold = 10;  // consecutive identical reads

    // Previous encoder snapshots used for identity comparison (distinct from the
    // velocity-compute snapshots _prevEncL/R above, which are updated only on
    // the refreshed wheel; these are updated every tick regardless of wheel).
    float    _wedgePrevEncL;
    float    _wedgePrevEncR;
    bool     _wedgePrevValidL;    // false until first reading
    bool     _wedgePrevValidR;

    uint8_t  _stuckCountL;        // consecutive identical-while-commanded reads (L)
    uint8_t  _stuckCountR;        // consecutive identical-while-commanded reads (R)
    bool     _wedgeEmittedL;      // latch: EVT already sent for this episode (L)
    bool     _wedgeEmittedR;      // latch: EVT already sent for this episode (R)

    // Optional I2CBus pointer for stats in EVT body.
    I2CBus*  _i2cBus;

    // Live pointers into LoopScheduler::activeFn / activeCtx so the EVT goes
    // to the channel that most recently sent a command.
    ReplyFn* _evtFn;
    void**   _evtCtx;

    // clamp helper
    static float clamp(float v, float lo, float hi);
};
