#pragma once

/**
 * RatioPidController — standard discrete PID with anti-windup integral clamp.
 *
 * Used by MotorController to compute faster-wheel correction in the
 * cumulative-distance ratio PID algorithm.
 *
 * The `integral` field is public so the slower-wheel adjustment in
 * MotorController can read it directly without a getter.
 */
class RatioPidController {
public:
    RatioPidController(float kP, float kI, float kD, float iClamp);

    /**
     * Compute one PID step.
     * @param error  Normalized error (dimensionless fraction).
     * @param dtS    Elapsed time since last call in seconds.
     * @return       Correction in PWM% units.
     */
    float update(float error, float dtS);

    /** Reset integrator and derivative state. Call on new command start. */
    void reset();

    /** Update PID gains at runtime (called by K-command setters). */
    void updateGains(float kP, float kI, float kD, float iClamp);

    float integral;  // public — read by slower-wheel adjustment logic

private:
    float _kP;
    float _kI;
    float _kD;
    float _iClamp;
    float _prevError;
    bool  _firstCall;

    static float clamp(float v, float lo, float hi);
};
