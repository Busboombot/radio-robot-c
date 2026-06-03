#pragma once
#include <stdint.h>
#include <math.h>
#include "Config.h"
#include "Protocol.h"

class MotorController;
class Odometry;
class OtosSensor;

/**
 * DriveController — owns and advances the S/T/D/G drive state machines,
 * S-mode watchdog, and odometry delta tracking.
 *
 * Calls MotorController for wheel control and reads Odometry for pose.
 * Calls Odometry::correct() on each slow-cadence OTOS sample when an
 * OtosSensor is connected (architecture: DC reads otos → Odo::correct).
 * Does not parse commands. Does not emit telemetry — telemetry is
 * assembled by Robot::tick() into a unified TLM frame.
 * Emits EVT completions (done, safety_stop) through the captured reply sink.
 *
 * Per-drive sink capture: each begin*() captures the originating reply
 * sink so that async completions (EVT done, EVT safety_stop) are returned
 * over the channel that initiated the drive, even if a later command arrives
 * on a different channel.
 */
class DriveController {
public:
    // otos may be nullptr if the OTOS sensor is not connected; correct() is skipped silently.
    DriveController(MotorController& mc, Odometry& odo, const RobotConfig& cfg,
                    OtosSensor* otos = nullptr);

    // Set or clear the OTOS sensor pointer (called by Robot after hardware probe).
    void setOtos(OtosSensor* otos) { _otos = otos; }

    // Entry points — called from Robot drive methods.
    // Each captures fn/ctx as the originating reply sink for async completions.
    // corr_id: originating command correlation id (digits only, no '#');
    //          nullptr or empty string when no id was supplied.
    void beginStream(float leftMms, float rightMms, uint32_t now_ms,
                     ReplyFn fn, void* ctx);
    void beginTimed(float leftMms, float rightMms, uint32_t durationMs, uint32_t now_ms,
                    ReplyFn fn, void* ctx, const char* corr_id = nullptr);
    void beginDistance(float leftMms, float rightMms, int32_t targetMm, uint32_t now_ms,
                       ReplyFn fn, void* ctx, const char* corr_id = nullptr);
    void beginGoTo(float tx, float ty, float speedMms, uint32_t now_ms,
                   ReplyFn fn, void* ctx, const char* corr_id = nullptr);
    void stop(uint32_t now_ms, ReplyFn fn, void* ctx);

    // Advance all state machines. Call once per main-loop iteration.
    // now_ms: current system time. fn/ctx: active-channel reply sink (for
    // completions if no per-drive sink was captured).
    void tick(uint32_t now_ms, ReplyFn fn, void* ctx);

    DriveMode mode() const { return _mode; }

private:
    MotorController&   _mc;
    Odometry&          _odo;
    const RobotConfig& _cfg;
    OtosSensor*        _otos;  // nullable; nullptr when OTOS not connected

    // Drive mode
    DriveMode _mode;

    // Captured per-drive reply sink — set when a drive begins; used for async
    // completions (EVT done, EVT safety_stop) so they return to the
    // channel that originated the drive command.
    ReplyFn  _driveFn;
    void*    _driveCtx;

    // Originating command correlation id (digits only, no '#').
    // Stored when T/D/G begin; appended to EVT done and EVT safety_stop.
    // Empty string when no id was supplied.
    char     _corrId[16];

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
    enum class GPhase { IDLE, PRE_ROTATE, PURSUE };
    GPhase _gPhase;
    float  _gTargetXWorld;  // goal x in world frame (mm), set at beginGoTo()
    float  _gTargetYWorld;  // goal y in world frame (mm), set at beginGoTo()
    float  _gSpeed;
    float  _vRamped;        // current ramped speed (mm/s); reset to 0 at beginGoTo() and PRE_ROTATE→PURSUE

    // Tick timing
    uint32_t _lastTickMs;

    // Updated at top of tick()
    uint32_t _currentTimeMs;

    // Slow-cadence OTOS polling: run correct() every kOtosSlowMs milliseconds.
    // OTOS is the slow optical sensor; predict() runs every fast tick.
    static constexpr uint32_t kOtosSlowMs = 100; // 10 Hz OTOS correction cadence
    uint32_t _lastOtosMs;  // timestamp of last OTOS correct() call

    // Internal helpers
    void fullStop(ReplyFn fn, void* ctx);
    void getPoseFloat(float& x, float& y, float& h_rad) const;
};
