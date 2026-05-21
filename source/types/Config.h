#pragma once
#include <stdint.h>

struct CalibParams {
    float mmPerDegL;
    float mmPerDegR;
    float kFF;
    float kScaleLF;
    float kScaleLB;
    float kScaleRF;
    float kScaleRB;
    float kAdjThreshold;
    float kAdjGain;
    float trackwidthMm;
    float ratioPidKp;
    float ratioPidKi;
    float ratioPidKd;
    float ratioPidMax;
    float turnThresholdMm;
    float doneTolMm;
};

inline CalibParams defaultCalibParams() {
    CalibParams p{};
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
    p.turnThresholdMm = 5.0f;
    p.doneTolMm       = 3.0f;
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
