// StopCondition.cpp — evaluate() for each Kind of stop condition.
//
// See StopCondition.h for full API and param layout documentation.
// Architecture reference: .clasi/sprints/017-.../architecture-update.md §StopCondition
// Sprint 017, Ticket 003.

#include "StopCondition.h"
#include <math.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// M_PI guard — micro:bit / ARMCC may not define M_PI by default.
// ---------------------------------------------------------------------------
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ---------------------------------------------------------------------------
// rgbToHSV — convert RGBC raw sensor values to HSV floats.
//
// R,G,B are raw uint16_t sensor counts. We normalise by C (clear/ambient)
// to get [0,1] floating-point channels, then convert to HSV.
//
// h returned in [0, 360); s,v in [0, 1].
// If C == 0 (dark), returns h=0, s=0, v=0.
// ---------------------------------------------------------------------------
static void rgbToHSV(uint16_t rRaw, uint16_t gRaw, uint16_t bRaw, uint16_t cRaw,
                     float& h, float& s, float& v)
{
    if (cRaw == 0) { h = 0.0f; s = 0.0f; v = 0.0f; return; }
    float r = (float)rRaw / (float)cRaw;
    float g = (float)gRaw / (float)cRaw;
    float b = (float)bRaw / (float)cRaw;
    // Clamp to [0,1].
    if (r > 1.0f) r = 1.0f;
    if (g > 1.0f) g = 1.0f;
    if (b > 1.0f) b = 1.0f;

    float cmax = r; if (g > cmax) cmax = g; if (b > cmax) cmax = b;
    float cmin = r; if (g < cmin) cmin = g; if (b < cmin) cmin = b;
    float delta = cmax - cmin;
    v = cmax;
    s = (cmax > 0.0f) ? (delta / cmax) : 0.0f;
    if (delta < 1e-6f) {
        h = 0.0f;
        return;
    }
    if (cmax == r) {
        h = 60.0f * ((g - b) / delta);
    } else if (cmax == g) {
        h = 60.0f * (((b - r) / delta) + 2.0f);
    } else {
        h = 60.0f * (((r - g) / delta) + 4.0f);
    }
    if (h < 0.0f) h += 360.0f;
}

// ---------------------------------------------------------------------------
// hueDistance — wrap-aware angular distance between two hue values [0,360).
// Returns a value in [0, 180].
// ---------------------------------------------------------------------------
static float hueDistance(float h1, float h2)
{
    float d = h1 - h2;
    if (d < 0.0f) d = -d;
    if (d > 180.0f) d = 360.0f - d;
    return d;
}

// ---------------------------------------------------------------------------
// wrap_angle — wrap x into (-π, π].
//
// Uses atan2f(sinf(x), cosf(x)) which is numerically robust and correct for
// all finite x without requiring a branch-heavy manual approach.
// ---------------------------------------------------------------------------
static float wrap_angle(float x)
{
    return atan2f(sinf(x), cosf(x));
}

// ---------------------------------------------------------------------------
// getSensorValue — map a SENSOR channel selector to a HardwareState value.
//
// Channel encoding (matches StopCondition.h doc):
//   0–3   : line[0..3]
//   4     : colorR
//   5     : colorG
//   6     : colorB
//   7     : colorC
//   8–11  : analogIn[0..3]
//   other : 0 (safe default)
// ---------------------------------------------------------------------------
static float getSensorValue(const HardwareState& s, uint8_t channel)
{
    if (channel < 4) {
        return static_cast<float>(s.line[channel]);
    }
    switch (channel) {
        case 4:  return static_cast<float>(s.colorR);
        case 5:  return static_cast<float>(s.colorG);
        case 6:  return static_cast<float>(s.colorB);
        case 7:  return static_cast<float>(s.colorC);
        case 8:  return static_cast<float>(s.analogIn[0]);
        case 9:  return static_cast<float>(s.analogIn[1]);
        case 10: return static_cast<float>(s.analogIn[2]);
        case 11: return static_cast<float>(s.analogIn[3]);
        default: return 0.0f;
    }
}

// ---------------------------------------------------------------------------
// StopCondition::evaluate
// ---------------------------------------------------------------------------

bool StopCondition::evaluate(const HardwareState& s, uint32_t now_ms,
                              const MotionBaseline& base) const
{
    switch (kind) {

        case Kind::NONE:
            return false;

        case Kind::TIME: {
            // `a` holds the threshold in milliseconds.
            // Use signed delta to guard against uint32 underflow when now_ms is
            // momentarily less than t0Ms (same pattern as driveAdvance watchdog).
            int32_t elapsed = (int32_t)(now_ms - base.t0Ms);
            return elapsed >= (int32_t)a;
        }

        case Kind::DISTANCE: {
            // `a` holds the distance threshold in mm.
            // Uses raw encoder sum (not filtered) per architecture decision:
            //   "filtered value can stall under outlier filtering" (D-command finding).
            float enc_avg = (s.encLMm + s.encRMm) * 0.5f;
            float traveled = enc_avg - base.enc0Mm;
            if (traveled < 0.0f) traveled = -traveled;  // fabsf without including math.h twice
            return traveled >= a;
        }

        case Kind::HEADING: {
            // `a` = target heading delta (rad); `b` = eps (rad).
            // Fires when the robot's heading is within eps of the target heading.
            // wrap_angle keeps the difference in (-π, π].
            float current_delta = wrap_angle(s.poseHrad - base.heading0Rad);
            float error = wrap_angle(current_delta - a);
            float abs_error = (error < 0.0f) ? -error : error;
            return abs_error < b;
        }

        case Kind::POSITION: {
            // `ax` = target X mm; `a` = target Y mm; `b` = radius mm.
            // Fires when the Euclidean distance from current pose to target is < b.
            float dx = s.poseX - ax;
            float dy = s.poseY - a;
            float dist2 = dx * dx + dy * dy;
            return dist2 < (b * b);
        }

        case Kind::SENSOR: {
            // `a` = threshold; `sensor` = channel; `cmp` = GE or LE.
            float val = getSensorValue(s, sensor);
            if (cmp == Cmp::GE) {
                return val >= a;
            } else {
                return val <= a;
            }
        }

        case Kind::COLOR: {
            // `a`  = target hue [0,360); `b` = target saturation [0,1];
            // `ay` = target value [0,1]; `ax` = HSV distance threshold.
            // Fires when sqrt(hDist^2 + sDist^2 + vDist^2) <= ax.
            float h, sv, v;
            rgbToHSV(s.colorR, s.colorG, s.colorB, s.colorC, h, sv, v);
            float hd = hueDistance(h, a);
            float sd = sv - b;
            float vd = v  - ay;
            float dist = sqrtf(hd * hd + sd * sd + vd * vd);
            return dist <= ax;
        }

        case Kind::LINE_ANY: {
            // `a` = threshold; `cmp` = GE or LE.
            // Fires on short-circuit OR across all 4 line channels.
            for (int i = 0; i < 4; ++i) {
                float val = static_cast<float>(s.line[i]);
                if (cmp == Cmp::GE) {
                    if (val >= a) return true;
                } else {
                    if (val <= a) return true;
                }
            }
            return false;
        }

        case Kind::ROTATION: {
            // `a` = target per-wheel arc (mm). For a spin the wheels move in
            // opposite directions, so the encoder DIFFERENTIAL (encR - encL)
            // tracks rotation while the sum (used by DISTANCE) stays ~0.
            // Per-wheel arc = |Δdiff| / 2.  Uses raw encoder values (not
            // filtered) — same rationale as DISTANCE: the filter can stall.
            float diff = (s.encRMm - s.encLMm) - base.encDiff0Mm;
            if (diff < 0.0f) diff = -diff;
            return (diff * 0.5f) >= a;
        }
    }

    // Unreachable; silence compiler warnings.
    return false;
}
