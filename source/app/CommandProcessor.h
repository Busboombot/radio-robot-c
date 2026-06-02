#pragma once
#include <stdint.h>
#include <math.h>
#include "Config.h"
#include "NezhaV2.h"
#include "MotorController.h"
#include "Odometry.h"
#include "Protocol.h"

// Forward declarations for optional peripherals (may be null).
class OtosSensor;
class LineSensor;
class ColorSensor;
class GripperServo;
class PortIO;

/**
 * CommandProcessor — wire-protocol parser and drive-mode state machine.
 *
 * Owns DriveMode state, S-mode watchdog, and streaming encoder output.
 * Calls MotorController (motor control) and Odometry (dead-reckoning).
 * Does NOT interact with hardware directly.
 *
 * Usage:
 *   CommandProcessor cmd;
 *   cmd.init(&motor, &mc, &odo, nullptr, nullptr, nullptr, nullptr, nullptr);
 *   // in tick loop:
 *   cmd.process(lineBuf, replyFn, ctx);
 *   cmd.tick(uBit.systemTime(), replyFn, ctx);
 */
class CommandProcessor {
public:
    CommandProcessor();

    // Inject hardware pointers. mc and odo must not be null. Others may be null.
    void init(NezhaV2*         motor,
              MotorController* mc,
              Odometry*        odo,
              OtosSensor*      otos,
              LineSensor*      line,
              ColorSensor*     color,
              GripperServo*    gripper,
              PortIO*          portio);

    // Set unified config pointer (call after init, from Robot.cpp).
    void setConfig(RobotConfig* config);

    // Parse and dispatch one command line. line must be NUL-terminated.
    // Calls replyFn(msg, ctx) for each response line.
    void process(const char* line, ReplyFn replyFn, void* ctx);

    // Drive-mode state machine tick. Call once per iteration of the main loop.
    // now_ms: current system time in ms (from uBit.systemTime()).
    // replyFn/ctx: same callback used by process().
    void tick(uint32_t now_ms, ReplyFn replyFn, void* ctx);

private:
    // Injected pointers
    NezhaV2*         _motor;
    MotorController* _mc;
    Odometry*        _odo;
    OtosSensor*      _otos;
    LineSensor*      _line;
    ColorSensor*     _color;
    GripperServo*    _gripper;
    PortIO*          _portio;
    RobotConfig*     _config;

    // Drive mode state
    DriveMode _mode;
    uint32_t  _lastSMs;       // time of last S command (for watchdog)
    float     _tgtL;          // current left target mm/s
    float     _tgtR;          // current right target mm/s

    // T-command termination
    uint32_t  _tEndMs;

    // D-command termination
    int32_t   _dEncStartL;    // mm at D command start
    int32_t   _dEncStartR;
    int32_t   _dTargetMm;
    uint32_t  _dTimeoutMs;

    // G go-to state machine
    enum class GPhase { IDLE, PRE_ROTATE, ARC };
    GPhase    _gPhase;
    float     _gTargetX;
    float     _gTargetY;
    float     _gSpeed;
    float     _gArcLeftMm;
    float     _gArcRightMm;
    float     _gArcStartL;
    float     _gArcStartR;

    // Streaming state
    int32_t   _encTickCount;  // counts up to encReportEvery

    // Tick timing
    uint32_t  _lastTickMs;

    // Current time (updated at top of tick, used by process handlers)
    uint32_t  _currentTimeMs;

    // Previous encoder positions for odometry delta computation
    int32_t   _prevOdoEncL;
    int32_t   _prevOdoEncR;

    // Gripper state
    int32_t   _currentGripperAngle;  // last angle sent to gripper (degrees, 0..180)

    // Internal helpers
    static int  parseSignedArgs(const char* s, int32_t* out, int maxArgs);
    static int  clampInt(int v, int lo, int hi);
    static int  clampMinSpeed(int mms, int minSpeedMms);
    static void computeArc(float tx, float ty, float trackwidthMm,
                           float& leftMm, float& rightMm);
    void        fullStop(ReplyFn replyFn, void* ctx);
    void        reportEncoders(ReplyFn replyFn, void* ctx);
    void        reportOdo(ReplyFn replyFn, void* ctx);
};
