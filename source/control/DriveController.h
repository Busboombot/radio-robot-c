#pragma once
#include <stdint.h>
#include <math.h>
#include <stddef.h>
#include "Config.h"
#include "Protocol.h"
#include "RobotState.h"

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
 * assembled by Robot::telemetryTick() into a unified TLM frame.
 *
 * Two-fiber architecture (013-010):
 *   - controlTick() runs on the high-priority control fiber.  It calls
 *     the motor/PID path and enqueues EVT completions into a small ring
 *     buffer.  It does NOT call any reply fn (no serial/radio I/O).
 *   - drainEvents(fn, ctx) is called from the comms+telemetry fiber
 *     (Robot::telemetryTick()) to pop and emit pending EVT messages.
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

    // Bind the authoritative HardwareState (called by Robot after state init,
    // before the first tick).  Required so Odometry::correct() can write the
    // struct-based pose fields (014-004).  Ticket 005 moves correct() into
    // Robot::otosCorrect() and removes this dependency.
    void setHardwareState(HardwareState* s) { _hwState = s; }

    // Entry points — called from Robot drive methods.
    // Each captures fn/ctx as the originating reply sink for async completions.
    // corr_id: originating command correlation id (digits only, no '#');
    //          nullptr or empty string when no id was supplied.
    void beginStream(float leftMms, float rightMms, uint32_t now_ms,
                     ReplyFn fn, void* ctx);
    // VW command entry point: converts (v, ω) body-twist to (vL, vR) via
    // BodyKinematics::inverse() + saturate(), then delegates to beginStream()
    // so the existing STREAMING watchdog handles keepalive/safety_stop.
    // corr_id: originating correlation id (digits only, no '#'); stored for
    //          EVT safety_stop when the watchdog fires.
    void beginVelocity(float v_mms, float omega_rads, uint32_t now_ms,
                       ReplyFn fn, void* ctx, const char* corr_id = nullptr);
    void beginTimed(float leftMms, float rightMms, uint32_t durationMs, uint32_t now_ms,
                    ReplyFn fn, void* ctx, const char* corr_id = nullptr);
    void beginDistance(float leftMms, float rightMms, int32_t targetMm, uint32_t now_ms,
                       ReplyFn fn, void* ctx, const char* corr_id = nullptr);
    void beginGoTo(float tx, float ty, float speedMms, uint32_t now_ms,
                   ReplyFn fn, void* ctx, const char* corr_id = nullptr);
    void stop(uint32_t now_ms, ReplyFn fn, void* ctx);

    // Control-fiber entry point (013-010): advance motor/PID state machines
    // only.  Does NOT call any reply fn — completions are enqueued into the
    // internal ring buffer for later drain by the comms fiber.
    void controlTick(uint32_t now_ms);

    // Drain pending EVT completions (safety_stop, done T/D/G) into fn/ctx.
    // Called from the comms+telemetry fiber (Robot::telemetryTick()).
    // Returns the number of events emitted.
    int drainEvents(ReplyFn fn, void* ctx);

    DriveMode mode() const { return _mode; }

private:
    MotorController&   _mc;
    Odometry&          _odo;
    const RobotConfig& _cfg;
    OtosSensor*        _otos;     // nullable; nullptr when OTOS not connected
    HardwareState*     _hwState;  // authoritative state for Odometry::correct() (014-004)

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

    // Updated at top of controlTick()
    uint32_t _currentTimeMs;

    // ---------------------------------------------------------------------------
    // EVT completion ring buffer (013-010)
    //
    // The control fiber enqueues EVT strings here instead of calling the reply fn
    // directly.  The comms fiber drains the queue via drainEvents().
    //
    // CODAL cooperative scheduler: on this single-core chip a fiber switch only
    // happens at explicit yield points (fiber_sleep / uBit.sleep).  The control
    // fiber never yields mid-enqueue and the comms fiber never yields mid-drain,
    // so a simple lock-free ring is safe (no mutex needed on the hot path).
    //
    // Ring capacity: 4 entries is ample — at most one completion fires per tick,
    // and the comms fiber drains every ~5 ms.
    // ---------------------------------------------------------------------------
    static constexpr int kEvtQueueCap = 4;
    struct EvtEntry {
        char msg[48];    // full EVT string including " #<corr_id>" suffix
        // captured reply sink (the channel that originated the drive command)
        ReplyFn fn;
        void*   ctx;
    };
    EvtEntry _evtQueue[kEvtQueueCap];
    int      _evtHead;   // index of next slot to write
    int      _evtTail;   // index of next slot to read (head == tail → empty)

    // Enqueue an EVT message with the current _corrId suffix.
    // Called only from the control fiber.
    void enqueueEvt(const char* base);

    // Slow-cadence OTOS polling: run correct() every kOtosSlowMs milliseconds.
    // OTOS is the slow optical sensor; predict() runs every fast tick.
    static constexpr uint32_t kOtosSlowMs = 100; // 10 Hz OTOS correction cadence
    uint32_t _lastOtosMs;  // timestamp of last OTOS correct() call

    // Internal helpers
    void fullStop(ReplyFn fn, void* ctx);
    void getPoseFloat(float& x, float& y, float& h_rad) const;
};
