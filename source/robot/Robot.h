#pragma once
#include "Config.h"
#include "Hardware.h"
#include "IMotor.h"
#include "IOtosSensor.h"
#include "ILineSensor.h"
#include "IColorSensor.h"
#include "IServo.h"
#include "IPortIO.h"
#include "MotorController.h"
#include "Odometry.h"
#include "MotionController.h"
#include "PortController.h"
#include "ServoController.h"
#include "RobotState.h"
#include "Protocol.h"
#include "../types/CommandTypes.h"
#include "../robot/ConfigRegistry.h"
#include "../control/HaltController.h"
#include "MotionCommandHandlers.h"

// Forward declarations — keeps the header-graph shallow.
class DebugCommandable;
class LoopScheduler;
struct Robot;

// ---------------------------------------------------------------------------
// RobotSysCtx — context bundle for system command handlers (HELLO, PING, …).
// handlerCtx for system CommandDescriptors is a RobotSysCtx*.
// ---------------------------------------------------------------------------
struct RobotSysCtx {
    Robot*         robot;
    LoopScheduler* sched;
};

/**
 * Robot — open struct that owns and wires all robot firmware subsystems.
 *
 * Replaces the Robot facade class (sprint 016).  All subsystem members are
 * public: there are no invariants to protect at this level — each subsystem
 * class protects its own.  Caller code reaches subsystems directly instead
 * of going through forwarding methods.
 *
 * Hardware is provided through a Hardware& (NezhaHAL on device, MockHAL in
 * host tests). Robot binds interface refs from hal.motorL() etc.
 * The control layer (MotorController, Odometry, MotionController) and state
 * (RobotConfig, RobotStateContainer) are VALUE MEMBERS owned by Robot.
 *
 * Member declaration order is load-bearing (C++ initialises members in
 * declaration order):
 *   1. hal ref                 — must be first (initialised before interface refs)
 *   2. config, state           — owned values; state needs config
 *   3. motorL, motorR refs     — bound before motorController constructs
 *   4. otos, line, color, gripper, portio refs
 *   5. motorController         — needs motorL, motorR, config refs
 *   6. odometry                — default ctor
 *   7. motionController        — needs motorController, odometry, config
 *   8. portController          — needs portio ref
 */
struct Robot {
    // ---- HAL reference (must be declared first) ----
    Hardware&           hal;

    // ---- Owned value members (initialized after hal) ----
    RobotConfig         config;   // owned copy; SET commands mutate this
    RobotStateContainer state;    // = defaultInputs(config)

    // ---- Device interface references (bound from hal in constructor) ----
    // Declared before motorController so the refs are live when motorController
    // constructs and binds motorL/motorR.
    IMotor&             motorL;
    IMotor&             motorR;
    IOtosSensor&        otos;
    ILineSensor&        line;
    IColorSensor&       colorSensor;  // named colorSensor to avoid macro collisions
    IServo&             gripper;
    IPortIO&            portio;

    // ---- Owned control-layer members (depend on refs above) ----
    MotorController     motorController;   // (motorL, motorR, config)
    Odometry            odometry;          // default ctor
    MotionController    motionController;  // (motorController, odometry, config)
    PortController      portController;    // (portio)
    ServoController     servoController;   // (gripper)
    HaltController      haltController;    // user-facing named stop-condition registry

    // ---- Constructor ----
    explicit Robot(Hardware& hal, const RobotConfig& cfg);

    // ---- Kept orchestration methods ----
    // These methods span multiple subsystems and are kept as Robot members
    // rather than inlined at every call site.

    // controlCollectSplitPhase — read both encoders, apply outlier filter,
    // write state.inputs.enc{L,R}Mm, then call motorController.controlTick().
    void controlCollectSplitPhase(uint32_t now_ms, int pendingWheel);

    // otosCorrect — read OTOS device, write state.inputs.otos*, call odometry.correct().
    // Uses otos.readTransformed(config) from T001 — no inlined LSB math.
    void otosCorrect(uint32_t now_ms);

    // Sensor read task entry points (write to state.inputs.*VS).
    void lineRead();
    void colorRead();
    void portsRead();

    // resetEncoders — atomically resets ALL encoder state so that both the outlier
    // filter baseline and Odometry's previous-encoder snapshot see a fresh zero.
    //
    // Atomically:
    //   1. Calls motorController.resetEncoderAccumulators() — resets hardware
    //      accumulators AND MotorController velocity baselines (_prevEncL/R,
    //      _hasTimestamp*, _prevTimeMsL/R).
    //   2. Zeroes state.inputs.encLMm / encRMm — aligns the outlier filter
    //      baseline with the fresh accumulators.
    //   3. Calls odometry.rebaselinePrev(0, 0) — prevents Odometry::predict()
    //      from computing a large negative delta (dL = 0 - _prevEncL) on the
    //      very next tick, which previously teleported the pose backward by the
    //      prior segment's travel.
    //
    // Does NOT touch pose (x, y, theta) or the EKF state.
    // Called by distanceDrive() (D command) and handleZero() (ZERO enc path).
    // (N1 fix, sprint 030-001.)
    void resetEncoders();

    // distanceDrive — calls motionController.beginDistance + calls resetEncoders()
    // to atomically reset hardware accumulators, velocity baselines, the outlier
    // filter baseline, and Odometry's encoder snapshot.
    void distanceDrive(int32_t l, int32_t r, int32_t targetMm,
                       ReplyFn fn, void* ctx, const char* corr_id = nullptr);

    // buildTlmFrame — assemble unified TLM frame; shared by STREAM and SNAP.
    int  buildTlmFrame(char* buf, int len);

    // telemetryEmit — gate and emit the periodic TLM frame.
    void telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx);

    // systemTime — robot system time in ms since boot.
    uint32_t systemTime() const;

    // setMotionQueue — bind the CommandQueue for VW converter push_front.
    // Called by LoopScheduler (or test harness) after the queue is created.
    // Null (default) causes converter handlers to fall back to direct begin*() calls.
    // Replaces motionController.setQueue() which was removed in sprint 026-002.
    void setMotionQueue(CommandQueue* q) { _motionCtx.queue = q; }

    // ---- Command-table building ----
    // Aggregate all command descriptors into a vector:
    //   Commandable members (motionController, odometry, portController,
    //   servoController), optional DebugCommandable, then system commands
    //   (HELLO, PING, ECHO, ID, VER, HELP, SNAP, ZERO, STREAM, RF,
    //    GET VEL, GET, SET).
    // sched may be nullptr; RF will reply ERR noradio if it is.
    std::vector<CommandDescriptor> buildCommandTable(
        DebugCommandable* dbg   = nullptr,
        LoopScheduler*    sched = nullptr) const;

    // ---- Bench OTOS tick (sprint 031) ----
    // Feed commanded velocity into BenchOtosSensor each control tick.
    // No-op (fast return) when bench mode is off or hal is not NezhaHAL.
    void benchOtosTick(uint32_t now_ms);

    // Enable/disable bench OTOS mode. Firmware delegates to NezhaHAL::setOtosBench;
    // HOST_BUILD records _simBenchOtosActive so the sim can observe the toggle.
    void setBenchOtosEnabled(bool on);

    // Returns true when the bench sensor is selected (NezhaHAL::isBenchMode in
    // firmware; the recorded flag in HOST_BUILD).
    bool isBenchOtosActive() const;

    // ---- Gating state that pairs with the kept methods ----
    uint32_t _lastTlmMs     = 0;
    uint32_t _lastActiveMs  = 0;
    uint32_t _lastControlMs = 0;
    bool     _prevDriving   = false;
    bool     _simBenchOtosActive = false;  // HOST_BUILD bench-mode mirror (033-002)

    // ---- Wedge-state tracking for enc-omega gate (033-005e) ----
    // Tracks whether a wheel was wedged on the previous tick so Robot can
    // restore setEncOmegaHealthy(true) on the tick the wedge clears.
    bool     _prevAnyWedged   = false;

    // ---- Outlier-filter hold instrumentation (033-005b) ----
    // Per-wheel consecutive-reject streak counters. Incremented each tick that
    // a wheel's encoder read is rejected by the speed-scaled outlier gate; reset
    // to 0 on any accepted read or when the robot is not driving. When either
    // streak reaches kFilterRejectStreakThreshold, an EVT enc_filter_hold line
    // is emitted (once per episode) to alert the host to a silent filter freeze.
    static constexpr uint8_t kFilterRejectStreakThreshold = 3;
    uint8_t  _filterRejectStreakL = 0;
    uint8_t  _filterRejectStreakR = 0;

    // ---- D10 telemetry: sequence counter + channel binding (028-005) ----
    // _tlmSeq: monotonically incrementing uint16 emitted as seq=<n> in every
    //   TLM frame (both STREAM and SNAP share the same counter).  Wraps at 65535.
    // _tlmBoundFn / _tlmBoundCtx: the reply channel bound by the last STREAM
    //   command.  Set in handleStream; nullptr means no STREAM has been issued
    //   (TLM is suppressed, same behaviour as tlmPeriodMs=0 on init).
    uint16_t _tlmSeq        = 0;
    ReplyFn  _tlmBoundFn    = nullptr;
    void*    _tlmBoundCtx   = nullptr;

    // ---- OTOS validity tracking (D9 — 027-005) ----
    // _otosInvalidStartMs: system time when OTOS first became invalid in the
    //   current invalidity window (0 = OTOS is currently valid / no window open).
    // _otosLostEmitted: true once "EVT otos lost" has been emitted for the
    //   current invalidity window; reset to false when OTOS becomes valid again.
    uint32_t _otosInvalidStartMs = 0;
    bool     _otosLostEmitted    = false;

private:
    // Stable storage for command contexts; pointers into these are placed in
    // CommandDescriptors, which must outlive the CommandProcessor.
    mutable CfgCtx      _cfgCtx    = {};  // GET / SET
    mutable RobotSysCtx _sysCtx    = {};  // HELLO, PING, ECHO, ID, VER, …, RF
    mutable MotionCtx   _motionCtx = {};  // S/T/D/G/R/TURN/RT/VW/X/STOP handlers (sprint 026-002)

    // Bench OTOS tick timestamp — for signed-delta dt computation.
    // Initialized to 0; first benchOtosTick() call passes dt=0 to tick() (no-op).
    uint32_t _lastBenchTickMs = 0;

    // CommandQueue forward declaration for setMotionQueue.
    // (Included via MotionCommandHandlers.h → CommandQueue.h.)
};
