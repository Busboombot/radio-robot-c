#pragma once
#include <stdint.h>
#include "Config.h"
#include "Protocol.h"
#include "MotionEventSink.h"

// ---------------------------------------------------------------------------
// ValueSet — freshness / validity envelope for a sensor group.
//
// lagMs      : expected sensor latency in ms; initialised from RobotConfig.
// lastUpdMs  : system time (ms) of the most recent valid reading.
// valid      : true once at least one reading has been received.
// ---------------------------------------------------------------------------
struct ValueSet {
    uint32_t lagMs;
    uint32_t lastUpdMs;
    bool     valid;
};

// ---------------------------------------------------------------------------
// MotorCommands — actuator outputs produced by the control loop.
// ---------------------------------------------------------------------------
struct MotorCommands {
    float    tgtLMms;           // left-wheel target speed, mm/s
    float    tgtRMms;           // right-wheel target speed, mm/s
    int16_t  pwmL;              // raw PWM output, left motor
    int16_t  pwmR;              // raw PWM output, right motor
    bool     digitalOut[4];     // digital output channels 0–3
    int16_t  analogOut[4];      // analogue output channels 0–3
    bool     digitalDirty[4];   // channel has unsent update
    bool     analogDirty[4];    // channel has unsent update
};

// ---------------------------------------------------------------------------
// HardwareState — all sensor readings (latest values + freshness envelopes).
// ---------------------------------------------------------------------------
struct HardwareState {
    // Encoder odometry
    float    encLMm;    // left-wheel cumulative distance, mm
    float    encRMm;    // right-wheel cumulative distance, mm
    ValueSet enc;       // freshness for encoder readings

    // Derived wheel velocities
    float    velLMms;   // left-wheel velocity, mm/s
    float    velRMms;   // right-wheel velocity, mm/s

    // Dead-reckoning pose (updated each encoder tick)
    float    poseX;     // robot X position, mm
    float    poseY;     // robot Y position, mm
    float    poseHrad;  // robot heading, radians
    ValueSet pose;      // freshness for pose (updated with enc)

    // EKF fused velocity (Sprint 023 — written by Odometry::predict and correctEKF)
    float    fusedV;       // body-frame linear speed, mm/s (from 5-state EKF)
    float    fusedOmega;   // yaw rate, rad/s (from 5-state EKF)

    // OTOS acceleration (passthrough; written by Robot::otosCorrect — Sprint 023)
    float    otosAccelX;   // forward acceleration, mm/s^2
    float    otosAccelY;   // lateral acceleration, mm/s^2

    // OTOS optical odometry sensor
    float    otosX;     // OTOS X reading, mm
    float    otosY;     // OTOS Y reading, mm
    float    otosH;     // OTOS heading, radians
    ValueSet otos;      // freshness for OTOS readings

    // Line sensor (4-channel)
    uint16_t line[4];   // raw line sensor values, channels 0–3
    ValueSet lineVS;    // freshness for line readings

    // Color sensor (RGBC)
    uint16_t colorR;    // red channel
    uint16_t colorG;    // green channel
    uint16_t colorB;    // blue channel
    uint16_t colorC;    // clear/ambient channel
    ValueSet colorVS;   // freshness for color readings

    // General-purpose I/O ports
    bool     digitalIn[4];  // digital input channels 0–3
    int16_t  analogIn[4];   // analogue input channels 0–3
    ValueSet portsVS;       // freshness for port readings
};

// ---------------------------------------------------------------------------
// TargetState — the current drive command from the radio / command processor.
// ---------------------------------------------------------------------------
struct TargetState {
    DriveMode mode;             // active drive mode (IDLE, STREAMING, …)
    float     targetXWorld;     // go-to target X in world frame, mm
    float     targetYWorld;     // go-to target Y in world frame, mm
    float     targetSpeedMms;   // requested travel speed, mm/s
    float     distanceTargetMm; // remaining distance for DISTANCE mode, mm
    uint32_t  deadlineMs;       // wall-clock deadline for TIMED mode, ms
    ReplyFn   replyFn;          // callback to send drive-complete reply
    void*     replyCtx;         // opaque context for replyFn
    char      corrId[16];       // correlation ID for the pending command
    MotionEventSink sink;       // narrow event sink for async EVT completions (sprint 026-002)
};

// ---------------------------------------------------------------------------
// RobotStateContainer — single authoritative state blob passed through the
// cooperative main loop (replaces per-subsystem private caches).
// ---------------------------------------------------------------------------
struct RobotStateContainer {
    MotorCommands commands;
    HardwareState inputs;
    TargetState   target;
};

// ---------------------------------------------------------------------------
// defaultInputs — zero-initialise the container, then seed each ValueSet's
// lagMs from the corresponding RobotConfig lag field.
// ---------------------------------------------------------------------------
inline RobotStateContainer defaultInputs(const RobotConfig& cfg) {
    RobotStateContainer s{};
    s.inputs.otos.lagMs   = cfg.lagOtosMs;
    s.inputs.lineVS.lagMs = cfg.lagLineMs;
    s.inputs.colorVS.lagMs = cfg.lagColorMs;
    s.inputs.portsVS.lagMs = cfg.lagPortsMs;
    // enc / pose lag: encoder readings are always synchronous in the control
    // loop, so lagMs is left at 0 (zero-initialised).
    return s;
}
