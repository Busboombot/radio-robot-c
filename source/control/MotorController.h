#pragma once
#include "MicroBit.h"
#include "Motor.h"
#include "Config.h"
#include "RatioPidController.h"
#include "VelocityController.h"

/**
 * MotorController — per-wheel velocity PID wheel speed control.
 *
 * Inner loop is VelocityController (PI+FF) — one instance per wheel.
 * RatioPidController is retained but bypassed (not called in normal drive).
 *
 * Sprint 010 replaces the cumulative-distance ratio PID inner loop with
 * two independent VelocityController instances (_vcL, _vcR) that track
 * per-wheel mm/s setpoints. See docs/kinematics-model.md §2.1.
 *
 * Thread safety: single-threaded tick loop only.
 */
class MotorController {
public:
    MotorController(Motor& left, Motor& right, const RobotConfig& cal);

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
    // Reads encoders, runs ratio PID + FF, clamps output, calls Motor::setSpeed().
    void tick(float dt_s);

    // Read actual wheel velocities in mm/s (encoder delta since last tick).
    void getActualVelocity(float& leftMms, float& rightMms) const;

    /**
     * getVelocitySourceFlags — report which velocity source is live per wheel.
     *
     * leftChip  = true if chip readSpeed (0x47) is the active source for left wheel.
     * rightChip = true if chip readSpeed (0x47) is the active source for right wheel.
     * false = encoder-delta fallback is in use (I2C error or implausibility gate).
     */
    void getVelocitySourceFlags(bool& leftChip, bool& rightChip) const;

    // Read cumulative encoder positions in mm (sum since last resetEncoderAccumulators()).
    void getEncoderPositions(int32_t& leftMm, int32_t& rightMm) const;

    // Zero encoder accumulators — delegates to Motor::resetEncoder() for each wheel.
    void resetEncoderAccumulators();

private:
    Motor&             _motorL;
    Motor&             _motorR;
    const RobotConfig& _cal;

    // Per-wheel velocity controllers (PI + feed-forward). Sprint 010 inner loop.
    VelocityController _vcL;
    VelocityController _vcR;

    // RatioPidController retained for compile compatibility; bypassed in normal drive.
    RatioPidController _pid;

    // Ratio-PID bookkeeping fields — retained but unused in velocity-loop path.
    float _cmdEncStartL;     // encoder mm snapshot at command start (left)
    float _cmdEncStartR;     // encoder mm snapshot at command start (right)
    float _cmdRatio;         // |fasterSpeed| / |slowerSpeed|, always >= 1.0
    bool  _fasterIsRight;    // true if right wheel is the commanded-faster wheel

    float _tgtLMms;          // current speed targets in mm/s
    float _tgtRMms;

    // Cached encoder readings from the most recent tick() call.
    float   _prevEncL;   // mm at start of last tick (float — high-res for velocity)
    float   _prevEncR;
    float   _actualVelL; // mm/s computed in tick()
    float   _actualVelR;
    float   _encLMm;     // current encoder position (mm), updated in tick()
    float   _encRMm;

    // Velocity source flags: true = chip readSpeed (0x47), false = encoder-delta fallback.
    bool _usingChipVelL;
    bool _usingChipVelR;

    // Read encoder and convert to mm.
    float encoderMm(bool left);

    // clamp helper
    static float clamp(float v, float lo, float hi);
};
