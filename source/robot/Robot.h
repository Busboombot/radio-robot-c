#pragma once
#include "Config.h"
#include "Motor.h"
#include "OtosSensor.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "Servo.h"
#include "PortIO.h"
#include "MotorController.h"
#include "Odometry.h"
#include "MotionController.h"
#include "PortController.h"
#include "RobotState.h"
#include "Protocol.h"

/**
 * Robot — open struct that owns and wires all robot firmware subsystems.
 *
 * Replaces the Robot facade class (sprint 016).  All subsystem members are
 * public: there are no invariants to protect at this level — each subsystem
 * class protects its own.  Caller code reaches subsystems directly instead
 * of going through forwarding methods.
 *
 * Devices (Motor, OtosSensor, LineSensor, ColorSensor, Servo, PortIO) are
 * constructed in main() as statics and held here as REFERENCES (not owned).
 * The control layer (MotorController, Odometry, MotionController) and state
 * (RobotConfig, RobotStateContainer) are VALUE MEMBERS owned by Robot.
 *
 * Member declaration order is load-bearing (C++ initialises members in
 * declaration order):
 *   1. config, state           — owned values; state needs config
 *   2. motorL, motorR refs     — bound before motorController constructs
 *   3. otos, line, color, gripper, portio refs
 *   4. motorController         — needs motorL, motorR, config refs
 *   5. odometry                — default ctor
 *   6. motionController        — needs motorController, odometry, config
 *   7. portController          — needs portio ref
 */
struct Robot {
    // ---- Owned value members (initialized first) ----
    RobotConfig         config;   // owned copy; SET commands mutate this
    RobotStateContainer state;    // = defaultInputs(config)

    // ---- Device references (bound in constructor, not owned) ----
    // Declared before motorController so the refs are live when motorController
    // constructs and binds motorL/motorR.
    Motor&              motorL;
    Motor&              motorR;
    OtosSensor&         otos;
    LineSensor&         line;
    ColorSensor&        colorSensor;  // named colorSensor to avoid macro collisions
    Servo&              gripper;
    PortIO&             portio;

    // ---- Owned control-layer members (depend on refs above) ----
    MotorController     motorController;   // (motorL, motorR, config)
    Odometry            odometry;          // default ctor
    MotionController    motionController;  // (motorController, odometry, config)
    PortController      portController;    // (portio)

    // ---- Constructor ----
    Robot(Motor& mL, Motor& mR, OtosSensor& o, LineSensor& l,
               ColorSensor& c, Servo& g, PortIO& p,
               const RobotConfig& cfg);

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

    // distanceDrive — calls motionController.beginDistance + zeroes encoder baseline
    // in state.inputs so the outlier filter tracks from 0 (encoder-reset workaround).
    void distanceDrive(int32_t l, int32_t r, int32_t targetMm,
                       ReplyFn fn, void* ctx, const char* corr_id = nullptr);

    // buildTlmFrame — assemble unified TLM frame; shared by STREAM and SNAP.
    int  buildTlmFrame(char* buf, int len);

    // telemetryEmit — gate and emit the periodic TLM frame.
    void telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx);

    // systemTime — robot system time in ms since boot.
    uint32_t systemTime() const;

    // ---- Gating state that pairs with the kept methods ----
    uint32_t _lastTlmMs     = 0;
    uint32_t _lastActiveMs  = 0;
    uint32_t _lastControlMs = 0;
    bool     _prevDriving   = false;
};
