#pragma once
#include "MicroBit.h"
#include "NezhaV2.h"
#include "Config.h"
#include "RatioPidController.h"

/**
 * MotorController — cumulative-distance ratio PID wheel speed control.
 *
 * Sprint 4 replaces the PI+FF tick() body with a ratio PID algorithm that
 * tracks cumulative encoder distance since the command started and keeps
 * the ratio of left:right distance equal to the ratio of commanded speeds.
 *
 * Thread safety: single-threaded tick loop only.
 */
class MotorController {
public:
    explicit MotorController(NezhaV2& motor, const CalibParams& cal);

    // Gains — public so CommandProcessor can update via K-commands.
    // Defaults: kFF=0.15, kP=0.05, kI=0.20, iClamp=60, kRatio=0.01
    struct Gains {
        float kFF;      // feed-forward coefficient
        float kP;       // proportional gain
        float kI;       // integral gain
        float iClamp;   // integral windup clamp (PWM units, ±)
        float kRatio;   // ratio cross-coupling gain (sprint 2 stub, small)
    } gains;

    // Set speed targets in mm/s. Zero both to coast (not brake).
    void setTarget(float leftMms, float rightMms);

    /**
     * startDriveClean — used by T, D, and G commands.
     * Full clean start: snapshot encoders, compute ratio, reset PID.
     * Always call this when starting a new bounded command.
     */
    void startDriveClean(float leftMms, float rightMms);

    /**
     * startDrive — used by the S (streaming) command only.
     * Re-seeds cmdEncStart to preserve accumulated ratio history across keepalive re-sends.
     * Does NOT reset PID unless the faster/slower assignment changes.
     */
    void startDrive(float leftMms, float rightMms);

    // Stop: zero targets, reset PID, and write zero PWM.
    void stop();

    // Reset integrators only (called by CommandProcessor on mode change).
    void resetIntegrators();

    // Update PID gains at runtime (called by K-command setters).
    void updatePidGains(float kP, float kI, float kD, float iClamp);

    // Run one control tick. dt_s is elapsed seconds since last tick.
    // Reads encoders, runs ratio PID + FF, clamps output, calls NezhaV2::setPwm().
    void tick(float dt_s);

    // Read actual wheel velocities in mm/s (encoder delta since last tick).
    void getActualVelocity(float& leftMms, float& rightMms) const;

    // Read cumulative encoder positions in mm (sum since last resetEncoderAccumulators()).
    void getEncoderPositions(int32_t& leftMm, int32_t& rightMm) const;

    // Zero encoder accumulators — delegates to NezhaV2::resetEncoders().
    void resetEncoderAccumulators();

private:
    NezhaV2&           _motor;
    const CalibParams& _cal;

    // Ratio PID state
    RatioPidController _pid;
    float _cmdEncStartL;     // encoder mm snapshot at command start (left)
    float _cmdEncStartR;     // encoder mm snapshot at command start (right)
    float _cmdRatio;         // |fasterSpeed| / |slowerSpeed|, always >= 1.0
    bool  _fasterIsRight;    // true if right wheel is the commanded-faster wheel
    float _tgtLMms;          // current speed targets in mm/s
    float _tgtRMms;

    // Cached encoder readings from the most recent tick() call.
    // Used to compute velocity and expose via getActualVelocity().
    int32_t _prevEncL;   // mm at start of last tick
    int32_t _prevEncR;
    float   _actualVelL; // mm/s computed in tick()
    float   _actualVelR;

    // Read encoder and convert to mm.
    float encoderMm(bool left);

    // clamp helper
    static float clamp(float v, float lo, float hi);
};
