#pragma once
#include <stdint.h>

/**
 * Odometry — differential-drive dead-reckoning pose tracker.
 *
 * Internal state is float for accuracy; protocol output is integer.
 * Heading convention: 0 = +X axis, positive = CCW (standard math).
 * Output convention: centidegrees (360 degrees = 36000 cdeg).
 *
 * Caller (CommandProcessor) must call update() once per tick with
 * the encoder deltas for that tick in mm.
 */
class Odometry {
public:
    Odometry();

    // Integrate one tick's wheel travel.
    // dL_mm, dR_mm: signed mm traveled by left and right wheels this tick.
    // trackwidthMm: distance between wheel contact points in mm.
    void update(float dL_mm, float dR_mm, float trackwidthMm);

    // Read current pose. x_mm and y_mm are integer mm; h_cdeg is
    // centidegrees (-18000..+18000 clamped).
    void getPose(int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg) const;

    // Overwrite pose (used by SI command).
    // h_cdeg is centidegrees; stored internally as radians.
    void setPose(int32_t x_mm, int32_t y_mm, int32_t h_cdeg);

    // Zero pose: equivalent to setPose(0, 0, 0).
    void zero();

private:
    float _x;          // mm, float internal
    float _y;          // mm, float internal
    float _headingRad; // radians

    static constexpr float PI_F        = 3.14159265f;
    static constexpr float RAD_TO_CDEG = 18000.0f / 3.14159265f;
    static constexpr float CDEG_TO_RAD = 3.14159265f / 18000.0f;
};
