#pragma once
#include <stdint.h>

struct RobotConfig {
    // Encoder calibration (mm per degree of motor rotation)
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

    // Go-to tolerances
    float turnThresholdMm;
    float doneTolMm;

    // Command scaling
    float   distScale;
    float   turnScale;

    // Timing and speed parameters
    int32_t minSpeedMms;
    int32_t tickMs;
    int32_t sTimeoutMs;
    int32_t encReportEvery;
};

inline RobotConfig defaultRobotConfig() {
    RobotConfig p{};
    p.mmPerDegL       = 0.487f;
    p.mmPerDegR       = 0.481f;
    p.kFF             = 0.15f;
    p.kScaleLF        = 1.0f;
    p.kScaleLB        = 1.0f;
    p.kScaleRF        = 1.0f;
    p.kScaleRB        = 1.0f;
    p.kAdjThreshold   = 0.5f;
    p.kAdjGain        = 0.05f;
    p.trackwidthMm    = 120.0f;
    p.ratioPidKp      = 300.0f;
    p.ratioPidKi      = 0.0f;
    p.ratioPidKd      = 0.0f;
    p.ratioPidMax     = 30.0f;
    p.turnThresholdMm = 50.0f;
    p.doneTolMm       = 5.0f;
    p.distScale       = 0.94f;
    p.turnScale       = 1.07f;
    p.minSpeedMms     = 50;
    p.tickMs          = 20;
    p.sTimeoutMs      = 200;
    p.encReportEvery  = 2;
    return p;
}

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
    GO_TO     = 4
};
