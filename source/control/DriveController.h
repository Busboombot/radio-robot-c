#pragma once
#include <stdint.h>
#include <math.h>
#include "Config.h"
#include "Protocol.h"

class MotorController;
class Odometry;

/**
 * DriveController — owns and advances the S/T/D/G drive state machines,
 * S-mode watchdog, streaming encoder counter, and odometry delta tracking.
 *
 * Calls MotorController for wheel control and reads Odometry for pose.
 * Does not own sensors. Does not parse commands.
 * Emits completion and telemetry strings through the injected ReplyFn.
 *
 * Per-drive sink capture: each begin*() captures the originating reply
 * sink so that async completions (T+DONE, D+DONE, G+DONE, SAFETY_STOP)
 * are returned over the channel that initiated the drive, even if a later
 * command arrives on a different channel.
 *
 * Sensor streaming: a SensorReportFn callback may be set via
 * setSensorReporter(). When set it is called alongside encoder reporting
 * each encReportEvery tick during an active drive.
 */

// Callback type for sensor streaming during drive ticks.
// The caller fills the line/color sensor readings into a string and
// writes it through the active reply sink.
using SensorReportFn = void(*)(ReplyFn fn, void* ctx, void* sensorCtx);

class DriveController {
public:
    DriveController(MotorController& mc, Odometry& odo, const RobotConfig& cfg);

    // Entry points — called from Robot drive methods.
    // Each captures fn/ctx as the originating reply sink for async completions.
    void beginStream(float leftMms, float rightMms, uint32_t now_ms,
                     ReplyFn fn, void* ctx);
    void beginTimed(float leftMms, float rightMms, uint32_t durationMs, uint32_t now_ms,
                    ReplyFn fn, void* ctx);
    void beginDistance(float leftMms, float rightMms, int32_t targetMm, uint32_t now_ms,
                       ReplyFn fn, void* ctx);
    void beginGoTo(float tx, float ty, float speedMms, uint32_t now_ms,
                   ReplyFn fn, void* ctx);
    void stop(uint32_t now_ms, ReplyFn fn, void* ctx);

    // Register a sensor streaming callback invoked alongside encoder reports.
    // sensorCtx is an opaque pointer passed back to the callback (Robot* typically).
    void setSensorReporter(SensorReportFn fn, void* sensorCtx);

    // Advance all state machines. Call once per main-loop iteration.
    // now_ms: current system time. fn/ctx: active-channel reply sink (used for
    // STREAMING mode sensor data; captured sink used for completions).
    void tick(uint32_t now_ms, ReplyFn fn, void* ctx);

    DriveMode mode() const { return _mode; }

private:
    MotorController&   _mc;
    Odometry&          _odo;
    const RobotConfig& _cfg;

    // Drive mode
    DriveMode _mode;

    // Captured per-drive reply sink — set when a drive begins; used for async
    // completions (T+DONE, D+DONE, G+DONE, SAFETY_STOP) so they return to the
    // channel that originated the drive command.
    ReplyFn  _driveFn;
    void*    _driveCtx;

    // Sensor streaming callback (optional)
    SensorReportFn _sensorFn;
    void*          _sensorCtx;

    // S-mode watchdog
    uint32_t _lastSMs;

    // Current speed targets (kept for internal use only)
    float _tgtL;
    float _tgtR;

    // T-command termination
    uint32_t _tEndMs;

    // D-command termination
    int32_t  _dEncStartL;
    int32_t  _dEncStartR;
    int32_t  _dTargetMm;
    uint32_t _dTimeoutMs;

    // G go-to state machine
    enum class GPhase { IDLE, PRE_ROTATE, ARC };
    GPhase _gPhase;
    float  _gTargetX;
    float  _gTargetY;
    float  _gSpeed;
    float  _gArcLeftMm;
    float  _gArcRightMm;
    float  _gArcStartL;
    float  _gArcStartR;

    // Streaming state
    int32_t _encTickCount;

    // Tick timing
    uint32_t _lastTickMs;

    // Updated at top of tick()
    uint32_t _currentTimeMs;

    // Previous encoder positions for odometry delta computation
    int32_t _prevOdoEncL;
    int32_t _prevOdoEncR;

    // Internal helpers
    void fullStop(ReplyFn fn, void* ctx);
    void reportEncoders(ReplyFn fn, void* ctx);
    void reportOdo(ReplyFn fn, void* ctx);

    static void computeArc(float tx, float ty, float trackwidthMm,
                           float& leftMm, float& rightMm);
};
