#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// Telemetry field bitmask constants (used in RobotConfig::tlmFields)
// ---------------------------------------------------------------------------
constexpr uint8_t TLM_FIELD_ENC   = (1u << 0);  // enc=l,r
constexpr uint8_t TLM_FIELD_POSE  = (1u << 1);  // pose=x,y,h
constexpr uint8_t TLM_FIELD_VEL   = (1u << 2);  // vel=vL,vR  (per-wheel mm/s, activated Sprint 010)
constexpr uint8_t TLM_FIELD_LINE  = (1u << 3);  // line=4ch
constexpr uint8_t TLM_FIELD_COLOR = (1u << 4);  // color=4ch
constexpr uint8_t TLM_FIELD_TWIST = (1u << 5);  // twist=v,omega  (fused body velocity, Sprint 023)
constexpr uint8_t TLM_FIELD_OTOS   = (1u << 6);  // otos=x,y,h  (raw OTOS pose mm/mm/cdeg, Sprint 023 bench)
constexpr uint8_t TLM_FIELD_EKFREJ = (1u << 7);  // ekf_rej=<n> (cumulative EKF rejection count, Sprint 024)
constexpr uint8_t TLM_FIELD_ALL    = 0xFFu;      // all fields (default)

struct RobotConfig {
    // Motor forward-direction signs: +1 = CW is forward, -1 = CCW is forward.
    // fwdSignL: left wheel (M2), default +1.
    // fwdSignR: right wheel (M1), default -1 (motor mounted mirrored).
    int8_t fwdSignL;
    int8_t fwdSignR;

    // Encoder calibration (mm per degree of motor rotation).
    // Also used by Motor::readSpeed() for chip-native velocity conversion:
    //   mm/s = (raw / kUnitFactor) * mmPerDeg * sign
    // (kUnitFactor is a named constant in Motor.cpp; see readSpeed() comment.)
    float mmPerDegL;
    float mmPerDegR;

    // Feed-forward and motor scale factors
    float kFF;
    float kScaleLF;
    float kScaleLB;
    float kScaleRF;
    float kScaleRB;

    // Slower-wheel adjustment
    float kAdjThreshold;
    float kAdjGain;

    // Geometry
    float trackwidthMm;

    // Ratio PID gains
    float ratioPidKp;
    float ratioPidKi;
    float ratioPidKd;
    float ratioPidMax;

    // Wheel saturation ceiling and steering headroom (docs/kinematics-model.md §1.7).
    // Effective ceiling = vWheelMax - steerHeadroom.
    // SET/GET key strings use dotted form: "vWheelMax", "steerHeadroom".
    float vWheelMax;      // absolute wheel speed ceiling, mm/s (default 400.0)
    float steerHeadroom;  // headroom below vWheelMax reserved for steering, mm/s (default 20.0)

    // Velocity controller gains (docs/kinematics-model.md §2.1).
    // C++ struct members cannot contain dots; SET/GET key strings use dotted form.
    //   velKp  ↔ key "vel.kP"   (default 0.3)
    //   velKi  ↔ key "vel.kI"   (default 0.05)
    //   velKff ↔ key "vel.kFF"  (default 0.15)
    float velKp;          // proportional gain for per-wheel velocity loop
    float velKi;          // integral gain for per-wheel velocity loop
    float velKff;         // feed-forward coefficient: FF = velKff * |setpoint|
    float minWheelMms;    // deadband: integrator frozen below this |speed| (default 20.0 mm/s)
    float velIMax;        // integrator clamp (PWM%, ±). SET "vel.iMax". Bounds windup.
    float velKaw;         // back-calc anti-windup gain (1/s). SET "vel.kAw". Bleeds the
                          // integrator while the output is saturated so a held wheel
                          // can't wind up → overshoot + slow recovery on release.
    // Velocity EMA filter + cross-wheel ratio coupling (SET keys "vel.filt", "sync").
    //   velFiltAlpha ↔ "vel.filt" : EMA weight on each new velocity sample
    //                  (1.0 = no filtering, lower = smoother/laggier; default 0.4)
    //   syncGain     ↔ "sync"     : cross-coupling gain. PWM correction
    //                  ±syncGain*(velL*ratio - velR) pulls the wheels onto the
    //                  commanded ratio line so disturbing one wheel slows the
    //                  other. 0 = independent wheels (old behavior); default 0.4
    float velFiltAlpha;
    float syncGain;

    // OTOS complementary fusion parameters (docs/kinematics-model.md §2.4).
    // C++ field names use flat camel-case; SET/GET key strings match exactly.
    //   alphaPos  — position blend gain, fraction of OTOS correction per slow tick (default 0.15)
    //   alphaYaw  — heading blend gain, fraction of OTOS correction per slow tick (default 0.10)
    //   otosGate  — outlier rejection threshold in mm; samples beyond this distance are dropped (default 50.0)
    float alphaPos;    // OTOS position blend gain [0, 1]
    float alphaYaw;    // OTOS heading blend gain [0, 1]
    float otosGate;    // outlier rejection distance threshold, mm

    // EKF sensor fusion parameters (sprint 022)
    float ekfQxy;       // process noise: position (mm^2) — default 2.0
    float ekfQtheta;    // process noise: heading (rad^2) — default 0.005
    float ekfROtosXy;   // OTOS measurement noise: position (mm^2) — default 50.0

    // EKF velocity fusion parameters (sprint 023)
    float ekfQv;          // process noise: body linear speed (mm^2/s^2) — default 50.0
    float ekfQomega;      // process noise: yaw rate (rad^2/s^2) — default 0.01
    float ekfROtosV;      // OTOS velocity measurement noise: body speed (mm^2/s^2) — default 200.0
    float ekfREncV;       // encoder velocity measurement noise: body speed (mm^2/s^2) — default 100.0

    // EKF heading fusion parameters (sprint 024-004)
    float ekfROtosTheta;  // OTOS heading measurement noise (rad^2) — default 0.01 ≈ (5.7°)²

    // OTOS calibration scalars and per-direction turn asymmetry (Sprint 012).
    // otosLinearScale: multiplier for OTOS linear calibration (e.g. 1.05).
    // otosAngularScale: multiplier for OTOS angular calibration (e.g. 0.987).
    // rotationGainPos: per-direction turn gain for CCW (positive) turns.
    // rotationGainNeg: per-direction turn gain for CW (negative) turns.
    // rotationOffsetDeg: turn offset added to CCW turns, degrees.
    // rotationOffsetDegNeg: turn offset added to CW turns, degrees.
    // rotationalSlip: body-rotation efficiency (arc / no-slip estimate).
    // odomOffX/odomOffY: OTOS mounting offset from robot center, mm.
    // odomYawDeg: OTOS mounting yaw offset, degrees.
    // odomUpsideDown: OTOS mounted upside-down (Z-axis flipped).
    float otosLinearScale;      // OTOS linear calibration multiplier (default 1.05)
    float otosAngularScale;     // OTOS angular calibration multiplier (default 0.987)
    float rotationGainPos;      // CCW turn gain (default 1.0)
    float rotationGainNeg;      // CW turn gain (default 1.17)
    float rotationOffsetDeg;    // CCW turn offset, degrees (default 0.0)
    float rotationOffsetDegNeg; // CW turn offset, degrees (default 0.0)
    float rotationalSlip;       // body-rotation efficiency (default 0.74)
    float odomOffX;             // OTOS X mounting offset, mm (default 0.0)
    float odomOffY;             // OTOS Y mounting offset, mm (default 0.0)
    float odomYawDeg;           // OTOS yaw mounting offset, degrees (default 0.0)
    bool  odomUpsideDown;       // OTOS mounted upside-down (default false)

    // Go-to tolerances (legacy, retained for backward compatibility)
    float turnThresholdMm;
    float doneTolMm;

    // Pose-control tunables (Sprint 011)
    // aMax: acceleration limit, mm/s²
    // aDecel: deceleration limit for v_cap, mm/s²
    // turnInPlaceGate: bearing threshold for in-place rotate, degrees on wire (default 45.0°)
    // arriveTolMm: go-to arrival tolerance, mm (float field, integer mm on wire)
    float aMax;
    float aDecel;
    float turnInPlaceGate;
    float arriveTolMm;

    // Body motion limits (Sprint 017 — BodyVelocityController).
    // vBodyMax:      body forward speed ceiling, mm/s          (default 400.0)
    // yawRateMax:    yaw rate ceiling, deg/s                   (default 180.0)
    // yawAccMax:     yaw acceleration limit, deg/s²            (default 720.0)
    // jMax:          linear jerk limit, mm/s³ (0 = trapezoid)  (default 0.0)
    // yawJerkMax:    yaw jerk limit, deg/s³   (0 = trapezoid)  (default 0.0)
    // aMax/aDecel above are reused for the linear channel (not duplicated).
    float vBodyMax;       // body forward speed ceiling, mm/s        (default 400.0)
    float yawRateMax;     // yaw rate ceiling, deg/s                 (default 180.0)
    float yawAccMax;      // yaw acceleration limit, deg/s²          (default 720.0)
    float jMax;           // linear jerk limit, mm/s³  (0=trapezoid) (default 0.0)
    float yawJerkMax;     // yaw jerk limit, deg/s³    (0=trapezoid) (default 0.0)

    // Command scaling
    float   distScale;
    float   turnScale;

    // Timing and speed parameters
    int32_t minSpeedMms;
    int32_t tickMs;
    int32_t sTimeoutMs;

    // System safety-stop watchdog enable. When false the watchdog never fires
    // (host keepalives not required) — for classroom T-driving where students
    // send self-terminating commands without streaming "+". Toggle at runtime
    // with the SAFE command (SAFE off / SAFE on [ms]). Default true.
    bool safetyEnabled;

    // Control fiber period in ms.  The control fiber (encoder reads → PID →
    // setSpeed) sleeps this many ms between iterations.  Distinct from tickMs
    // so the control rate can be tuned independently of the legacy tick cadence.
    // Default 10 ms → target ~100 Hz; actual rate depends on I2C busy-wait cost.
    int32_t controlPeriodMs;

    // Telemetry streaming period in ms (0 = off). Set via STREAM command.
    int32_t tlmPeriodMs;

    // Telemetry field-subscription bitmask. Set via STREAM fields=...
    // Bit 0 = enc, Bit 1 = pose, Bit 2 = vel, Bit 3 = line, Bit 4 = color.
    // 0xFF = all fields (default).
    uint8_t tlmFields;

    // One-shot SNAP pending flag. Set by SNAP command; cleared after one TLM frame.
    bool tlmSnapPending;

    // Sensor lag budgets (ms) used by RobotState freshness envelopes.
    // Each value is the expected worst-case latency for that sensor group.
    // lagOtosMs  : OTOS optical odometry sensor  (default 100 ms)
    // lagLineMs  : 4-channel line sensor          (default  50 ms)
    // lagColorMs : RGBC color sensor              (default 100 ms)
    // lagPortsMs : general-purpose I/O ports      (default  50 ms)
    uint32_t lagOtosMs;
    uint32_t lagLineMs;
    uint32_t lagColorMs;
    uint32_t lagPortsMs;
};

// Implemented in source/robot/DefaultConfig.cpp (auto-generated by
// scripts/gen_default_config.py from the active robot JSON config).
RobotConfig defaultRobotConfig();

struct MotorGains {
    float kp;
    float ki;
    float kff;
};

enum class DriveMode : uint8_t {
    IDLE      = 0,
    STREAMING = 1,
    TIMED     = 2,
    DISTANCE  = 3,
    GO_TO     = 4,
    VELOCITY  = 5   // MotionCommand-based body-twist control (Sprint 017)
};
