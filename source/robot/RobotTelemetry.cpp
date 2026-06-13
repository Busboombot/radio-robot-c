// ---------------------------------------------------------------------------
// RobotTelemetry.cpp — Robot::buildTlmFrame and Robot::telemetryEmit
//
// Split from Robot.cpp (sprint 035 A3). These are Robot:: member definitions
// only; class layout and Robot.h are unchanged.
// ---------------------------------------------------------------------------

#include "Robot.h"
#include <cstdio>
#include <cmath>

// ---------------------------------------------------------------------------
// buildTlmFrame — assemble the unified TLM frame; returns length.
//
// Reads state.inputs, config, motionController.mode(). Shared by the periodic
// STREAM (telemetryEmit) and the synchronous SNAP command.
// ---------------------------------------------------------------------------

int Robot::buildTlmFrame(char* buf, int len)
{
    uint32_t t_sample = systemTime();
    int32_t encL = static_cast<int32_t>(state.inputs.encLMm);
    int32_t encR = static_cast<int32_t>(state.inputs.encRMm);

    int32_t pose_x = 0, pose_y = 0, pose_h = 0;
    if (config.tlmFields & TLM_FIELD_POSE) {
        Odometry::getPose(state.inputs, pose_x, pose_y, pose_h);
    }
    // N8 (030-008): gate line/color on freshness, not just the sticky valid bit.
    // A sensor that wedges after boot keeps valid=true forever; consult the
    // lastUpdMs / lagMs envelope instead: fresh = now - lastUpdMs <= 2*lagMs.
    // lagMs is 0 until the first valid read (lastUpdMs stays 0 too), so the
    // subtraction wraps and the gate is never met -- correct for "never read".
    bool haveLine = line.is_initialized() &&
                    state.inputs.lineVS.valid &&
                    (t_sample - state.inputs.lineVS.lastUpdMs
                         <= 2u * state.inputs.lineVS.lagMs) &&
                    (config.tlmFields & TLM_FIELD_LINE);
    bool haveColor = colorSensor.is_initialized() &&
                     state.inputs.colorVS.valid &&
                     (t_sample - state.inputs.colorVS.lastUpdMs
                          <= 2u * state.inputs.colorVS.lagMs) &&
                     (config.tlmFields & TLM_FIELD_COLOR);
    bool haveVel = (config.tlmFields & TLM_FIELD_VEL) != 0;
    float velL = haveVel ? state.inputs.velLMms : 0.0f;
    float velR = haveVel ? state.inputs.velRMms : 0.0f;
    bool haveTwist = (config.tlmFields & TLM_FIELD_TWIST) != 0;

    char modeChar = 'I';
    switch (motionController.mode()) {
        case DriveMode::STREAMING: modeChar = 'S'; break;
        case DriveMode::DISTANCE:  modeChar = 'D'; break;
        case DriveMode::GO_TO:     modeChar = 'G'; break;
        case DriveMode::VELOCITY:  modeChar = 'V'; break;
        // N13 (030-010): TIMED removed -- T command runs as VELOCITY; mode=T
        // was unreachable in firmware. Host parser handles mode=T gracefully
        // for backward-compatibility with old logs.
        default:                   modeChar = 'I'; break;
    }

    int pos = 0, rem = len;
    int n = snprintf(buf + pos, (size_t)rem, "TLM t=%lu mode=%c seq=%u",
                     (unsigned long)t_sample, modeChar, (unsigned)_tlmSeq++);
    if (n > 0 && n < rem) { pos += n; rem -= n; }
    if (config.tlmFields & TLM_FIELD_ENC) {
        n = snprintf(buf + pos, (size_t)rem, " enc=%d,%d", (int)encL, (int)encR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (config.tlmFields & TLM_FIELD_POSE) {
        n = snprintf(buf + pos, (size_t)rem, " pose=%d,%d,%d",
                     (int)pose_x, (int)pose_y, (int)pose_h);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveVel) {
        n = snprintf(buf + pos, (size_t)rem, " vel=%d,%d", (int)velL, (int)velR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveTwist) {
        // fusedV is body linear speed in mm/s (integer).
        // fusedOmega is yaw rate in rad/s; convert to mrad/s (integer) matching
        // the omega_mrads convention used by VW command and NezhaProtocol.vw().
        n = snprintf(buf + pos, (size_t)rem, " twist=%d,%d",
                     (int)state.inputs.fusedV,
                     (int)(state.inputs.fusedOmega * 1000.0f));
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    // N8 (030-008): gate raw otos= on freshness -- same 2*lagMs rule as
    // line/color above.  otos.valid stays true after the first success; if the
    // sensor goes dark the last-good pose would be emitted forever without
    // the freshness check.
    if ((config.tlmFields & TLM_FIELD_OTOS) &&
        state.inputs.otos.valid &&
        (t_sample - state.inputs.otos.lastUpdMs
             <= 2u * state.inputs.otos.lagMs)) {
        // Raw OTOS pose (pre-fusion): x,y mm and heading in centidegrees,
        // matching the pose= field encoding. Lets the host plot the raw OTOS
        // sensor track alongside enc-derived and fused pose. 18000/pi cdeg/rad.
        n = snprintf(buf + pos, (size_t)rem, " otos=%d,%d,%d",
                     (int)state.inputs.otosX,
                     (int)state.inputs.otosY,
                     (int)(state.inputs.otosH * 5729.5779513f));
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveLine) {
        n = snprintf(buf + pos, (size_t)rem, " line=%u,%u,%u,%u",
                     (unsigned)state.inputs.line[0], (unsigned)state.inputs.line[1],
                     (unsigned)state.inputs.line[2], (unsigned)state.inputs.line[3]);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveColor) {
        n = snprintf(buf + pos, (size_t)rem, " color=%u,%u,%u,%u",
                     (unsigned)state.inputs.colorR, (unsigned)state.inputs.colorG,
                     (unsigned)state.inputs.colorB, (unsigned)state.inputs.colorC);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (config.tlmFields & TLM_FIELD_EKFREJ) {
        // Cumulative EKF gate rejection count -- all channels (pos, heading, velocity).
        // Sprint 024-005: emitted as ekf_rej=<n> for divergence visibility.
        n = snprintf(buf + pos, (size_t)rem, " ekf_rej=%d",
                     odometry.ekfRejectCount());
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    buf[pos] = '\0';
    return pos;
}

// ---------------------------------------------------------------------------
// telemetryEmit -- gate and emit the periodic TLM frame.
//
// D10 idle-rate change (028-005): the stream no longer goes silent when the
// robot is stopped.  When idle, the effective period is max(tlmPeriodMs, 500)
// so the host can distinguish "robot idle" from "serial dropped."
// The clamp (tlmPeriodMs < 20 -> 20) is enforced in handleStream, not here;
// telemetryEmit must NOT write to config.
// ---------------------------------------------------------------------------

void Robot::telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    if (config.tlmPeriodMs <= 0) return;

    // N3 null guard (030-003): _tlmBoundFn stays nullptr until STREAM binds the
    // channel.  SET tlmPeriod without a prior STREAM must not reach fn(...) -- a
    // null fn-pointer call is a HardFault on the micro:bit.  Silent suppression
    // matches the Robot.h:164-169 comment ("nullptr means TLM is suppressed").
    if (fn == nullptr) return;

    // Idle-rate: when stopped, slow down to max(period, 500 ms) so the stream
    // stays alive but doesn't flood the link with idle noise.
    static constexpr uint32_t kIdleMinMs = 500;
    static constexpr uint32_t kGraceMs   = 400;
    if (motionController.mode() != DriveMode::IDLE) _lastActiveMs = now_ms;
    bool stopped = ((now_ms - _lastActiveMs) > kGraceMs);

    uint32_t effectivePeriod = stopped
        ? ((uint32_t)config.tlmPeriodMs > kIdleMinMs
               ? (uint32_t)config.tlmPeriodMs
               : kIdleMinMs)
        : (uint32_t)config.tlmPeriodMs;

    if ((now_ms - _lastTlmMs) < effectivePeriod) return;

    char tlmBuf[160];
    buildTlmFrame(tlmBuf, sizeof(tlmBuf));
    fn(tlmBuf, ctx);
    _lastTlmMs = now_ms;
}
