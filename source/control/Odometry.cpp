#include "Odometry.h"
#include "CommandProcessor.h"
#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

Odometry::Odometry()
    : _prevEncL(0.0f), _prevEncR(0.0f)
    , _otosRejected(0)
    , _lastPredictMs(0)
    , _rOtosV(0.0f), _rEncV(0.0f), _rOtosTheta(0.0f)
    , _lastEncV(0.0f), _lastEncOmega(0.0f)
    , _odomCtx{this, nullptr}
{
}

// ---------------------------------------------------------------------------
// predict — midpoint (exact-arc) integration (docs/kinematics-model.md §2.4)
//
// Reads s.encLMm / s.encRMm; writes s.poseX / s.poseY / s.poseHrad.
// ---------------------------------------------------------------------------

void Odometry::predict(HardwareState& s, float trackwidthMm,
                       float rotationalSlip, uint32_t now_ms)
{
    // Compute dt — use signed cast to avoid uint32 underflow on rollover.
    // (See watchdog-uint32-underflow project finding: never plain-subtract
    //  two uint32 ms stamps without a signed cast.)
    float dt_s = 0.0f;
    if (_lastPredictMs == 0) {
        // First call: seed the timestamp, skip velocity update this tick.
        _lastPredictMs = now_ms;
    } else {
        dt_s = (int32_t)(now_ms - _lastPredictMs) * 0.001f;
    }

    float theta_before = s.poseHrad;   // heading before this step — MUST be first

    float dL = s.encLMm - _prevEncL;
    float dR = s.encRMm - _prevEncR;
    _prevEncL = s.encLMm;
    _prevEncR = s.encRMm;

    float dCenter   = (dL + dR) * 0.5f;
    // Apply rotational-slip correction: encoder arc over-reports body rotation
    // (wheel scrub during turns).  slip factor in [0.5, 1.0]; 0/unset → 1.0.
    // (024-006: rotationalSlip is now active — was dead before this sprint.)
    float slip      = effectiveSlip(rotationalSlip);
    float dTheta    = ((dR - dL) / trackwidthMm) * slip;
    float thetaMid  = s.poseHrad + dTheta * 0.5f;

    s.poseX    += dCenter * cosf(thetaMid);
    s.poseY    += dCenter * sinf(thetaMid);
    s.poseHrad  = wrapPi(s.poseHrad + dTheta);

    // Compute encoder-rate velocity for this tick.
    // Guard against dt_s == 0 (first tick or duplicate timestamp): skip velocity
    // computation and retain previous value (which is 0 on the very first tick).
    if (dt_s > 0.0f) {
        _lastEncV     = dCenter / dt_s;        // body linear speed (mm/s)
        _lastEncOmega = dTheta  / dt_s;        // yaw rate (rad/s)
    }
    // else: retain previous _lastEncV/_lastEncOmega (0 on first tick)

    // EKF predict — propagate state and covariance using encoder-derived arc segment.
    _ekf.predict(dCenter, dTheta, theta_before, dt_s);
    s.poseX    = _ekf.x();
    s.poseY    = _ekf.y();
    s.poseHrad = _ekf.theta();

    // Write EKF velocity states back to HardwareState.
    s.fusedV     = _ekf.v();
    s.fusedOmega = _ekf.omega();

    _lastPredictMs = now_ms;
}

// ---------------------------------------------------------------------------
// correct — OTOS complementary correction (docs/kinematics-model.md §2.4)
//
// Reads and writes s.poseX / s.poseY / s.poseHrad.
// ---------------------------------------------------------------------------

void Odometry::correct(HardwareState& s,
                       float x_otos, float y_otos, float theta_otos_rad,
                       float alphaPos, float alphaYaw, float otosGate)
{
    // Outlier gate: reject if OTOS position disagrees with predicted pose
    // by more than the gate threshold.
    float dx = x_otos - s.poseX;
    float dy = y_otos - s.poseY;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist > otosGate) {
        ++_otosRejected;
        return;
    }

    // Accepted: complementary blend of position.
    s.poseX += alphaPos * dx;
    s.poseY += alphaPos * dy;

    // Heading blend: angle-wrap-safe — blend on the angular difference,
    // not on the raw angle, to avoid crossing the ±π discontinuity.
    float dh = wrapPi(theta_otos_rad - s.poseHrad);
    s.poseHrad = wrapPi(s.poseHrad + alphaYaw * dh);
}

// ---------------------------------------------------------------------------
// getPose — read pose from s and convert to integer mm + centidegrees.
// ---------------------------------------------------------------------------

void Odometry::getPose(const HardwareState& s,
                       int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg)
{
    x_mm = static_cast<int32_t>(s.poseX);
    y_mm = static_cast<int32_t>(s.poseY);

    float cdeg = s.poseHrad * RAD_TO_CDEG;
    if (cdeg >  18000.0f) cdeg =  18000.0f;
    if (cdeg < -18000.0f) cdeg = -18000.0f;
    h_cdeg = static_cast<int32_t>(cdeg);
}

// ---------------------------------------------------------------------------
// setPose — write pose into s; also reset prev-encoder snapshot.
// ---------------------------------------------------------------------------

void Odometry::setPose(HardwareState& s, int32_t x_mm, int32_t y_mm, int32_t h_cdeg)
{
    s.poseX    = static_cast<float>(x_mm);
    s.poseY    = static_cast<float>(y_mm);
    s.poseHrad = static_cast<float>(h_cdeg) * CDEG_TO_RAD;

    // Re-baseline encoder snapshot to current encoder values (not 0.0f).
    // This prevents a spurious encoder-delta jump on the very next predict()
    // call after a camera fix (SI command) when encoders are non-zero.
    // Note: zero() calls setPose(s, 0, 0, 0) at startup when encoders read 0,
    // so _prevEncL = s.encLMm = 0 there — identical to the old behaviour on boot.
    _prevEncL  = s.encLMm;
    _prevEncR  = s.encRMm;

    _ekf.setPose(s.poseX, s.poseY, s.poseHrad);
}

// ---------------------------------------------------------------------------
// zero — reset pose to origin; reset prev-encoder snapshot.
// ---------------------------------------------------------------------------

void Odometry::zero(HardwareState& s)
{
    setPose(s, 0, 0, 0);
}

// ---------------------------------------------------------------------------
// update — legacy forward-Euler (deprecated; callers should use predict()).
// ---------------------------------------------------------------------------

void Odometry::update(HardwareState& s, float dL_mm, float dR_mm, float trackwidthMm)
{
    float dCenter = (dL_mm + dR_mm) * 0.5f;
    float dTheta  = (dR_mm - dL_mm) / trackwidthMm;

    s.poseX    += dCenter * cosf(s.poseHrad);
    s.poseY    += dCenter * sinf(s.poseHrad);
    s.poseHrad += dTheta;
}

// ---------------------------------------------------------------------------
// wrapPi — keep heading in (-π, π]
// ---------------------------------------------------------------------------

float Odometry::wrapPi(float theta)
{
    return atan2f(sinf(theta), cosf(theta));
}

// ---------------------------------------------------------------------------
// initEKF — set EKF process and measurement noise parameters.
// ---------------------------------------------------------------------------

void Odometry::initEKF(float q_xy, float q_theta, float q_v, float q_omega,
                       float r_otos_xy, float r_otos_v, float r_enc_v,
                       float r_otos_theta)
{
    _ekf.init(q_xy, q_theta, q_v, q_omega, r_otos_xy, r_otos_v, r_enc_v);
    // Cache the velocity noise params for use in correctEKF() calls.
    // _rOtosV is used for both v and omega of the OTOS source (symmetric
    // simplification — v1 design; separate v/omega noise is a future extension).
    _rOtosV     = r_otos_v;
    _rEncV      = r_enc_v;
    _rOtosTheta = r_otos_theta;  // OTOS heading noise (sprint 024-004)
}

// ---------------------------------------------------------------------------
// correctEKF — apply OTOS position, heading, and velocity observations to
// the EKF (sprint 024-004: heading fusion added).
//
// Update order: position → heading → velocity(OTOS) → velocity(enc).
// All channels are Mahalanobis-gated inside EKF methods.
// ---------------------------------------------------------------------------

void Odometry::correctEKF(HardwareState& s,
                          float x_otos, float y_otos,
                          float theta_otos_rad,
                          float v_otos_mmps, float omega_otos_rads,
                          float v_enc_mmps, float omega_enc_rads)
{
    // 1. Fuse OTOS position (Mahalanobis-gated inside EKF).
    _ekf.updatePosition(x_otos, y_otos);

    // 2. Fuse OTOS heading (sprint 024-004). H=[0,0,1,0,0]; wrap-safe innovation.
    _ekf.updateHeading(theta_otos_rad, _rOtosTheta);

    // 3. Fuse OTOS velocity (v, omega). Single scalar _rOtosV used for both
    //    v and omega noise (symmetric simplification — v1 design).
    _ekf.updateVelocity(v_otos_mmps, omega_otos_rads, _rOtosV, _rOtosV);

    // 4. Fuse encoder-derived velocity (v, omega). Similarly symmetric.
    _ekf.updateVelocity(v_enc_mmps, omega_enc_rads, _rEncV, _rEncV);

    // Write all EKF outputs back to HardwareState.
    s.poseX      = _ekf.x();
    s.poseY      = _ekf.y();
    s.poseHrad   = _ekf.theta();
    s.fusedV     = _ekf.v();
    s.fusedOmega = _ekf.omega();
}

// ===========================================================================
// Commandable implementation — OI, OZ, OR, OP, OV, OL, OA
//
// Each command mirrors the corresponding switch case in CommandProcessor.cpp.
// No behavior change — the old switch cases remain active until a future
// ticket removes them.
//
// Context type: OdomCtx* (cast from handlerCtx); all handlers use otos.
// ===========================================================================

// ---------------------------------------------------------------------------
// Parse functions — strip verb token so tokens[0] is the first argument.
// ---------------------------------------------------------------------------

// OI — no arguments
static ParseResult parseOI(const char* const* /*tokens*/, int /*ntokens*/,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

// OZ — no arguments
static ParseResult parseOZ(const char* const* /*tokens*/, int /*ntokens*/,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

// OR — no arguments
static ParseResult parseOR(const char* const* /*tokens*/, int /*ntokens*/,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

// OP — no arguments
static ParseResult parseOP(const char* const* /*tokens*/, int /*ntokens*/,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

// OV <x> <y> <h> — three mandatory int16 arguments
static ParseResult parseOV(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    if (ntokens < 3) {
        r.ok = false; r.err = { "badarg", nullptr }; return r;
    }
    r.ok = true;
    r.args.count = 3;
    r.args.args[0].type = ArgType::INT; r.args.args[0].ival = (int16_t)atoi(tokens[0]);
    r.args.args[1].type = ArgType::INT; r.args.args[1].ival = (int16_t)atoi(tokens[1]);
    r.args.args[2].type = ArgType::INT; r.args.args[2].ival = (int16_t)atoi(tokens[2]);
    return r;
}

// OL [val] — optional int8 scalar
static ParseResult parseOL(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true;
    if (ntokens >= 1) {
        r.args.count = 1;
        r.args.args[0].type = ArgType::INT;
        r.args.args[0].ival = (int8_t)atoi(tokens[0]);
    } else {
        r.args.count = 0;
    }
    return r;
}

// OA [val] — optional int8 scalar
static ParseResult parseOA(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true;
    if (ntokens >= 1) {
        r.args.count = 1;
        r.args.args[0].type = ArgType::INT;
        r.args.args[0].ival = (int8_t)atoi(tokens[0]);
    } else {
        r.args.count = 0;
    }
    return r;
}

// ---------------------------------------------------------------------------
// Handler functions
// ---------------------------------------------------------------------------

static void handleOI(const ArgList& /*args*/, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OdomCtx* c = reinterpret_cast<OdomCtx*>(handlerCtx);
    char rbuf[64];
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "oi",
                                   corrId, replyFn, replyCtx);
        return;
    }
    c->otos->init();
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "oi", nullptr,
                              corrId, replyFn, replyCtx);
}

static void handleOZ(const ArgList& /*args*/, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OdomCtx* c = reinterpret_cast<OdomCtx*>(handlerCtx);
    char rbuf[64];
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "oz",
                                   corrId, replyFn, replyCtx);
        return;
    }
    c->otos->setPositionRaw(0, 0, 0);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "oz", nullptr,
                              corrId, replyFn, replyCtx);
}

static void handleOR(const ArgList& /*args*/, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OdomCtx* c = reinterpret_cast<OdomCtx*>(handlerCtx);
    char rbuf[64];
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "or",
                                   corrId, replyFn, replyCtx);
        return;
    }
    c->otos->resetTracking();
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "or", nullptr,
                              corrId, replyFn, replyCtx);
}

// handleOP — report current OTOS pose from cached HardwareState.
//
// Reads hwState->otosX/Y/H (values written by Robot::otosCorrect() each OTOS
// task tick) instead of calling otos->getPositionRaw() on the device.
// This is the only Odometry command that does NOT access hardware (flag = CMD_NONE).
// If hwState is null (test harness without OTOS), returns zeros.
//
// Reply format: OK op x=<mm> y=<mm> h=<mrad>
//   x, y: OTOS position in integer mm.
//   h: OTOS heading in integer mrad (milliradians, for precision).
static void handleOP(const ArgList& /*args*/, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OdomCtx* c = reinterpret_cast<OdomCtx*>(handlerCtx);
    char rbuf[96];

    float x = 0.0f, y = 0.0f, h = 0.0f;
    if (c->hwState != nullptr) {
        x = c->hwState->otosX;
        y = c->hwState->otosY;
        h = c->hwState->otosH;
    }

    // Convert heading from radians to integer milliradians for the reply.
    int x_mm   = (int)x;
    int y_mm   = (int)y;
    int h_mrad = (int)(h * 1000.0f);

    char body[64];
    snprintf(body, sizeof(body), "x=%d y=%d h=%d", x_mm, y_mm, h_mrad);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "op", body,
                              corrId, replyFn, replyCtx);
}

static void handleOV(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OdomCtx* c = reinterpret_cast<OdomCtx*>(handlerCtx);
    char rbuf[96];
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "ov",
                                   corrId, replyFn, replyCtx);
        return;
    }
    int16_t ox = (int16_t)args.args[0].ival;
    int16_t oy = (int16_t)args.args[1].ival;
    int16_t oh = (int16_t)args.args[2].ival;
    c->otos->setPositionRaw(ox, oy, oh);
    char body[48];
    snprintf(body, sizeof(body), "x=%d y=%d h=%d", (int)ox, (int)oy, (int)oh);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "setpos", body,
                              corrId, replyFn, replyCtx);
}

static void handleOL(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OdomCtx* c = reinterpret_cast<OdomCtx*>(handlerCtx);
    char rbuf[64];
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "ol",
                                   corrId, replyFn, replyCtx);
        return;
    }
    if (args.count >= 1) {
        c->otos->setLinearScalar((int8_t)args.args[0].ival);
    }
    int8_t val = c->otos->getLinearScalar();
    char body[24];
    snprintf(body, sizeof(body), "scalar=%d", (int)val);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "linear", body,
                              corrId, replyFn, replyCtx);
}

static void handleOA(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OdomCtx* c = reinterpret_cast<OdomCtx*>(handlerCtx);
    char rbuf[64];
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "oa",
                                   corrId, replyFn, replyCtx);
        return;
    }
    if (args.count >= 1) {
        c->otos->setAngularScalar((int8_t)args.args[0].ival);
    }
    int8_t val = c->otos->getAngularScalar();
    char body[24];
    snprintf(body, sizeof(body), "scalar=%d", (int)val);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "angular", body,
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> Odometry::getCommands() const
{
    void* ctx = const_cast<OdomCtx*>(&_odomCtx);
    return {
        makeCmd("OI", parseOI, handleOI, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS init: re-initialise sensor
        makeCmd("OZ", parseOZ, handleOZ, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS zero: reset position to 0,0,0
        makeCmd("OR", parseOR, handleOR, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS read: one-shot position snapshot
        makeCmd("OP", parseOP, handleOP, ctx, "badarg"), // OTOS position: report current x,y,h (reads cached state)
        makeCmd("OV", parseOV, handleOV, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS velocity: report vx,vy,omega
        makeCmd("OL", parseOL, handleOL, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS linear scalar calibration
        makeCmd("OA", parseOA, handleOA, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS angular scalar calibration
    };
}
