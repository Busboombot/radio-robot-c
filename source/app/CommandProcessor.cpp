// CommandProcessor.cpp — wire-protocol parser and drive-mode state machine.
// C++ port of command.ts::handleCommand() and command.ts::tick().
//
// All speeds in mm/s, all distances in mm. Integer protocol, no floats on wire.
// Commands use sign-prefixed numbers as delimiters (no spaces). The legacy
// MakeCode relay capped lines at 19 chars; the RAW250 RadioRelay raises this to
// 250-byte messages, so command lines may be up to 250 bytes (see process()).

#include "CommandProcessor.h"
#include "OtosSensor.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "GripperServo.h"
#include "PortIO.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CommandProcessor::CommandProcessor()
    : _motor(nullptr)
    , _mc(nullptr)
    , _odo(nullptr)
    , _otos(nullptr)
    , _line(nullptr)
    , _color(nullptr)
    , _gripper(nullptr)
    , _portio(nullptr)
    , _cal(nullptr)
    , _mode(DriveMode::IDLE)
    , _lastSMs(0)
    , _tgtL(0.0f)
    , _tgtR(0.0f)
    , _tEndMs(0)
    , _dEncStartL(0)
    , _dEncStartR(0)
    , _dTargetMm(0)
    , _dTimeoutMs(0)
    , _gPhase(GPhase::IDLE)
    , _gTargetX(0.0f)
    , _gTargetY(0.0f)
    , _gSpeed(0.0f)
    , _gArcLeftMm(0.0f)
    , _gArcRightMm(0.0f)
    , _gArcStartL(0.0f)
    , _gArcStartR(0.0f)
    , _encTickCount(0)
    , _lastTickMs(0)
    , _currentTimeMs(0)
    , _prevOdoEncL(0)
    , _prevOdoEncR(0)
    , _currentGripperAngle(0)
{
    params.mmPerDegL      = 0.487f;
    params.mmPerDegR      = 0.481f;
    params.distScale      = 0.94f;
    params.turnScale      = 1.07f;
    params.minSpeedMms    = 50;
    params.tickMs         = 20;
    params.sTimeoutMs     = 200;
    params.encReportEvery = 2;
    params.trackwidthMm   = 120.0f;
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------

void CommandProcessor::init(NezhaV2*         motor,
                             MotorController* mc,
                             Odometry*        odo,
                             OtosSensor*      otos,
                             LineSensor*      line,
                             ColorSensor*     color,
                             GripperServo*    gripper,
                             PortIO*          portio)
{
    _motor   = motor;
    _mc      = mc;
    _odo     = odo;
    _otos    = otos;
    _line    = line;
    _color   = color;
    _gripper = gripper;
    _portio  = portio;
}

void CommandProcessor::setCalib(CalibParams* cal)
{
    _cal = cal;
}

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

/**
 * Parse sign-prefixed integer arguments from a string.
 * Example: "+200-150"  -> out[0]=200, out[1]=-150, returns 2
 * Example: "+200+200+1000" -> out[0]=200, out[1]=200, out[2]=1000, returns 3
 */
int CommandProcessor::parseSignedArgs(const char* s, int32_t* out, int maxArgs)
{
    int    count    = 0;
    bool   inNum    = false;
    bool   negative = false;
    int32_t accum   = 0;

    for (const char* p = s; *p != '\0' && count < maxArgs; ++p) {
        char ch = *p;
        if (ch == '+' || ch == '-') {
            if (inNum) {
                out[count++] = negative ? -accum : accum;
            }
            inNum    = true;
            negative = (ch == '-');
            accum    = 0;
        } else if (ch >= '0' && ch <= '9') {
            if (inNum) {
                accum = accum * 10 + (ch - '0');
            }
            // digits before any sign are ignored (shouldn't occur in protocol)
        }
    }
    if (inNum && count < maxArgs) {
        out[count++] = negative ? -accum : accum;
    }
    return count;
}

int CommandProcessor::clampInt(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int CommandProcessor::clampMinSpeed(int mms, int minSpeedMms)
{
    if (mms == 0) return 0;
    if (mms > 0 && mms < minSpeedMms) return minSpeedMms;
    if (mms < 0 && mms > -minSpeedMms) return -minSpeedMms;
    return mms;
}

/**
 * Compute differential arc wheel distances for a relative XY target.
 * Robot starts at (0,0,0). Heading=0 is forward (+X direction).
 *
 * @param tx           Target X in mm (forward from robot)
 * @param ty           Target Y in mm (left from robot)
 * @param trackwidthMm Distance between wheel contact patches in mm
 * @param leftMm       Output: left wheel distance in mm (signed)
 * @param rightMm      Output: right wheel distance in mm (signed)
 */
void CommandProcessor::computeArc(float tx, float ty, float trackwidthMm,
                                   float& leftMm, float& rightMm)
{
    float W = trackwidthMm;
    // Special case: ty == 0 means straight ahead
    if (fabsf(ty) < 0.001f) {
        leftMm  = tx;
        rightMm = tx;
        return;
    }
    float R     = (tx * tx + ty * ty) / (2.0f * ty);
    float alpha = atan2f(ty, tx + R);
    leftMm  = (R - W / 2.0f) * alpha;
    rightMm = (R + W / 2.0f) * alpha;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void CommandProcessor::fullStop(ReplyFn replyFn, void* ctx)
{
    _mc->stop();
    _mode         = DriveMode::IDLE;
    _tgtL         = 0.0f;
    _tgtR         = 0.0f;
    _encTickCount = 0;
    (void)replyFn;
    (void)ctx;
}

void CommandProcessor::reportEncoders(ReplyFn replyFn, void* ctx)
{
    int32_t l, r;
    _mc->getEncoderPositions(l, r);
    char buf[32];
    snprintf(buf, sizeof(buf), "ENC%+d%+d", (int)l, (int)r);
    replyFn(buf, ctx);
}

void CommandProcessor::reportOdo(ReplyFn replyFn, void* ctx)
{
    int32_t x, y, h;
    _odo->getPose(x, y, h);
    char buf[48];
    snprintf(buf, sizeof(buf), "SO%+d%+d%+d", (int)x, (int)y, (int)h);
    replyFn(buf, ctx);
}

// ---------------------------------------------------------------------------
// process — command dispatch
// ---------------------------------------------------------------------------

void CommandProcessor::process(const char* line, ReplyFn replyFn, void* ctx)
{
    // Copy to local uppercase buffer (250-byte RAW250 message max, NUL-terminated)
    char buf[256];
    int  len = 0;
    for (const char* p = line; *p != '\0' && len < 255; ++p) {
        char ch = *p;
        // Skip leading/trailing whitespace during copy
        if ((unsigned char)ch < 0x21 && len == 0) continue;  // skip leading whitespace
        buf[len++] = (char)toupper((unsigned char)ch);
    }
    // Trim trailing whitespace
    while (len > 0 && (unsigned char)buf[len - 1] < 0x21) --len;
    buf[len] = '\0';

    if (len == 0) return;

    // ── X or STOP — full stop ───────────────────────────────────────────────
    if ((len == 1 && buf[0] == 'X') ||
        (len == 4 && memcmp(buf, "STOP", 4) == 0)) {
        fullStop(replyFn, ctx);
        replyFn("ACK:X", ctx);
        return;
    }

    // ── S<L><R> — streaming drive ───────────────────────────────────────────
    // Must start with 'S' followed by '+' or '-' (not "SO", "SZ", "SI")
    if (buf[0] == 'S' && len > 1 && (buf[1] == '+' || buf[1] == '-')) {
        int32_t args[2] = {0, 0};
        int n = parseSignedArgs(buf + 1, args, 2);
        if (n < 2) {
            char errbuf[140];
            snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
            replyFn(errbuf, ctx);
            return;
        }
        int leftMms  = clampMinSpeed((int)args[0], (int)params.minSpeedMms);
        int rightMms = clampMinSpeed((int)args[1], (int)params.minSpeedMms);

        _mc->startDrive((float)leftMms, (float)rightMms);
        _mc->setTarget((float)leftMms, (float)rightMms);
        _tgtL    = (float)leftMms;
        _tgtR    = (float)rightMms;
        _mode    = DriveMode::STREAMING;
        _lastSMs = _currentTimeMs;

        char reply[48];
        snprintf(reply, sizeof(reply), "ACK:S %d %d", leftMms, rightMms);
        replyFn(reply, ctx);
        return;
    }

    // ── T<L><R><ms> — timed drive ──────────────────────────────────────────
    if (buf[0] == 'T' && len > 1) {
        int32_t args[3] = {0, 0, 0};
        int n = parseSignedArgs(buf + 1, args, 3);
        if (n < 3) {
            char errbuf[140];
            snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
            replyFn(errbuf, ctx);
            return;
        }
        int leftMms    = (int)args[0];
        int rightMms   = (int)args[1];
        int durationMs = clampInt((int)args[2], 1, 5000);

        _mc->startDriveClean((float)leftMms, (float)rightMms);
        _mc->setTarget((float)leftMms, (float)rightMms);
        _tgtL   = (float)leftMms;
        _tgtR   = (float)rightMms;
        _tEndMs = _lastTickMs + (uint32_t)durationMs;
        _mode   = DriveMode::TIMED;

        char reply[64];
        snprintf(reply, sizeof(reply), "ACK:T %d %d %d", leftMms, rightMms, durationMs);
        replyFn(reply, ctx);
        return;
    }

    // ── D<L><R><mm> — distance drive ───────────────────────────────────────
    if (buf[0] == 'D' && len > 1) {
        int32_t args[3] = {0, 0, 0};
        int n = parseSignedArgs(buf + 1, args, 3);
        if (n < 3) {
            char errbuf[140];
            snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
            replyFn(errbuf, ctx);
            return;
        }
        int leftMms  = (int)args[0];
        int rightMms = (int)args[1];
        int targetMm = abs((int)args[2]);
        if (targetMm < 1) targetMm = 1;

        _mc->startDriveClean((float)leftMms, (float)rightMms);
        _mc->setTarget((float)leftMms, (float)rightMms);
        _tgtL = (float)leftMms;
        _tgtR = (float)rightMms;
        _mc->resetEncoderAccumulators();
        _mc->getEncoderPositions(_dEncStartL, _dEncStartR);
        _dTargetMm  = targetMm;
        _dTimeoutMs = _lastTickMs + 5000;
        _mode       = DriveMode::DISTANCE;

        char reply[64];
        snprintf(reply, sizeof(reply), "ACK:D %d %d %d", leftMms, rightMms, targetMm);
        replyFn(reply, ctx);
        return;
    }

    // ── ENC — query encoder positions ──────────────────────────────────────
    if (len == 3 && memcmp(buf, "ENC", 3) == 0) {
        reportEncoders(replyFn, ctx);
        return;
    }

    // ── EZ — zero encoders ─────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "EZ", 2) == 0) {
        _mc->resetEncoderAccumulators();
        replyFn("ACK:EZ", ctx);
        return;
    }

    // ── SO — query odometry pose ────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "SO", 2) == 0) {
        reportOdo(replyFn, ctx);
        return;
    }

    // ── SZ — zero odometry ─────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "SZ", 2) == 0) {
        _odo->zero();
        replyFn("ACK:SZ", ctx);
        return;
    }

    // ── SI<x><y><h> — set odometry pose ────────────────────────────────────
    if (len > 2 && memcmp(buf, "SI", 2) == 0) {
        int32_t args[3] = {0, 0, 0};
        int n = parseSignedArgs(buf + 2, args, 3);
        if (n < 3) {
            char errbuf[140];
            snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
            replyFn(errbuf, ctx);
            return;
        }
        _odo->setPose(args[0], args[1], args[2]);
        char reply[64];
        snprintf(reply, sizeof(reply), "ACK:SI %d %d %d",
                 (int)args[0], (int)args[1], (int)args[2]);
        replyFn(reply, ctx);
        return;
    }

    // ── K — calibration dump or setter ─────────────────────────────────────
    if (buf[0] == 'K') {
        // K alone — dump all params
        if (len == 1) {
            char kbuf[32];
            snprintf(kbuf, sizeof(kbuf), "K:KML:%+d", (int)(params.mmPerDegL * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KMR:%+d", (int)(params.mmPerDegR * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            if (_cal) {
                snprintf(kbuf, sizeof(kbuf), "K:KFF:%+d", (int)(_cal->kFF * 1000.0f + 0.5f));
                replyFn(kbuf, ctx);
            }
            snprintf(kbuf, sizeof(kbuf), "K:KSM:%+d", (int)params.minSpeedMms);
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KSS:%+d", (int)params.sTimeoutMs);
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KTR:%+d", (int)params.tickMs);
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KER:%+d", (int)params.encReportEvery);
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KSD:%+d", (int)(params.distScale * 100.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KST:%+d", (int)(params.turnScale * 100.0f + 0.5f));
            replyFn(kbuf, ctx);
            if (_cal) {
                snprintf(kbuf, sizeof(kbuf), "K:KLF:%+d", (int)(_cal->kScaleLF * 1000.0f + 0.5f));
                replyFn(kbuf, ctx);
                snprintf(kbuf, sizeof(kbuf), "K:KLB:%+d", (int)(_cal->kScaleLB * 1000.0f + 0.5f));
                replyFn(kbuf, ctx);
                snprintf(kbuf, sizeof(kbuf), "K:KRF:%+d", (int)(_cal->kScaleRF * 1000.0f + 0.5f));
                replyFn(kbuf, ctx);
                snprintf(kbuf, sizeof(kbuf), "K:KRB:%+d", (int)(_cal->kScaleRB * 1000.0f + 0.5f));
                replyFn(kbuf, ctx);
                snprintf(kbuf, sizeof(kbuf), "K:KCP:%+d", (int)(_cal->ratioPidKp * 10.0f + 0.5f));
                replyFn(kbuf, ctx);
                snprintf(kbuf, sizeof(kbuf), "K:KCI:%+d", (int)(_cal->ratioPidKi * 1000.0f + 0.5f));
                replyFn(kbuf, ctx);
                snprintf(kbuf, sizeof(kbuf), "K:KCD:%+d", (int)(_cal->ratioPidKd * 1000.0f + 0.5f));
                replyFn(kbuf, ctx);
                snprintf(kbuf, sizeof(kbuf), "K:KCC:%+d", (int)(_cal->ratioPidMax));
                replyFn(kbuf, ctx);
                snprintf(kbuf, sizeof(kbuf), "K:KAT:%+d", (int)(_cal->kAdjThreshold * 1000.0f + 0.5f));
                replyFn(kbuf, ctx);
                snprintf(kbuf, sizeof(kbuf), "K:KAG:%+d", (int)(_cal->kAdjGain * 1000.0f + 0.5f));
                replyFn(kbuf, ctx);
                snprintf(kbuf, sizeof(kbuf), "K:KTW:%+d", (int)(_cal->trackwidthMm));
                replyFn(kbuf, ctx);
                snprintf(kbuf, sizeof(kbuf), "K:KGT:%+d", (int)(_cal->turnThresholdMm));
                replyFn(kbuf, ctx);
                snprintf(kbuf, sizeof(kbuf), "K:KGD:%+d", (int)(_cal->doneTolMm));
                replyFn(kbuf, ctx);
            }
            return;
        }

        // K setter — must be at least 4 chars: K + 2-char key + at least one sign/digit
        if (len >= 4) {
            // Extract 2-char key (chars 1..2) and parse 1 arg from chars 3+
            char key[3] = { buf[1], buf[2], '\0' };
            int32_t args[1] = {0};
            int n = parseSignedArgs(buf + 3, args, 1);
            if (n < 1) {
                char errbuf[140];
                snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
                replyFn(errbuf, ctx);
                return;
            }
            int v = (int)args[0];
            char reply[48];

            if (memcmp(key, "ML", 2) == 0) {
                params.mmPerDegL = v / 1000.0f;
                snprintf(reply, sizeof(reply), "ACK:KML %d", (int)(params.mmPerDegL * 1000.0f + 0.5f));
                replyFn(reply, ctx);
                return;
            }
            if (memcmp(key, "MR", 2) == 0) {
                params.mmPerDegR = v / 1000.0f;
                snprintf(reply, sizeof(reply), "ACK:KMR %d", (int)(params.mmPerDegR * 1000.0f + 0.5f));
                replyFn(reply, ctx);
                return;
            }
            if (_cal) {
                if (memcmp(key, "FF", 2) == 0) {
                    _cal->kFF = v / 1000.0f;
                    snprintf(reply, sizeof(reply), "ACK:KFF %d", (int)(_cal->kFF * 1000.0f + 0.5f));
                    replyFn(reply, ctx);
                    return;
                }
                if (memcmp(key, "LF", 2) == 0) {
                    _cal->kScaleLF = v / 1000.0f;
                    snprintf(reply, sizeof(reply), "ACK:KLF %d", (int)(_cal->kScaleLF * 1000.0f + 0.5f));
                    replyFn(reply, ctx); return;
                }
                if (memcmp(key, "LB", 2) == 0) {
                    _cal->kScaleLB = v / 1000.0f;
                    snprintf(reply, sizeof(reply), "ACK:KLB %d", (int)(_cal->kScaleLB * 1000.0f + 0.5f));
                    replyFn(reply, ctx); return;
                }
                if (memcmp(key, "RF", 2) == 0) {
                    _cal->kScaleRF = v / 1000.0f;
                    snprintf(reply, sizeof(reply), "ACK:KRF %d", (int)(_cal->kScaleRF * 1000.0f + 0.5f));
                    replyFn(reply, ctx); return;
                }
                if (memcmp(key, "RB", 2) == 0) {
                    _cal->kScaleRB = v / 1000.0f;
                    snprintf(reply, sizeof(reply), "ACK:KRB %d", (int)(_cal->kScaleRB * 1000.0f + 0.5f));
                    replyFn(reply, ctx); return;
                }
                if (memcmp(key, "CP", 2) == 0) {
                    _cal->ratioPidKp = v / 10.0f;
                    if (_mc) _mc->updatePidGains(_cal->ratioPidKp, _cal->ratioPidKi, _cal->ratioPidKd, _cal->ratioPidMax);
                    snprintf(reply, sizeof(reply), "ACK:KCP %d", (int)(_cal->ratioPidKp * 10.0f + 0.5f));
                    replyFn(reply, ctx); return;
                }
                if (memcmp(key, "CI", 2) == 0) {
                    _cal->ratioPidKi = v / 1000.0f;
                    if (_mc) _mc->updatePidGains(_cal->ratioPidKp, _cal->ratioPidKi, _cal->ratioPidKd, _cal->ratioPidMax);
                    snprintf(reply, sizeof(reply), "ACK:KCI %d", (int)(_cal->ratioPidKi * 1000.0f + 0.5f));
                    replyFn(reply, ctx); return;
                }
                if (memcmp(key, "CD", 2) == 0) {
                    _cal->ratioPidKd = v / 1000.0f;
                    if (_mc) _mc->updatePidGains(_cal->ratioPidKp, _cal->ratioPidKi, _cal->ratioPidKd, _cal->ratioPidMax);
                    snprintf(reply, sizeof(reply), "ACK:KCD %d", (int)(_cal->ratioPidKd * 1000.0f + 0.5f));
                    replyFn(reply, ctx); return;
                }
                if (memcmp(key, "CC", 2) == 0) {
                    _cal->ratioPidMax = (float)v;
                    if (_mc) _mc->updatePidGains(_cal->ratioPidKp, _cal->ratioPidKi, _cal->ratioPidKd, _cal->ratioPidMax);
                    snprintf(reply, sizeof(reply), "ACK:KCC %d", (int)_cal->ratioPidMax);
                    replyFn(reply, ctx); return;
                }
                if (memcmp(key, "AT", 2) == 0) {
                    _cal->kAdjThreshold = v / 1000.0f;
                    snprintf(reply, sizeof(reply), "ACK:KAT %d", (int)(_cal->kAdjThreshold * 1000.0f + 0.5f));
                    replyFn(reply, ctx); return;
                }
                if (memcmp(key, "AG", 2) == 0) {
                    _cal->kAdjGain = v / 1000.0f;
                    snprintf(reply, sizeof(reply), "ACK:KAG %d", (int)(_cal->kAdjGain * 1000.0f + 0.5f));
                    replyFn(reply, ctx); return;
                }
                if (memcmp(key, "TW", 2) == 0) {
                    _cal->trackwidthMm = (float)v;
                    snprintf(reply, sizeof(reply), "ACK:KTW %d", (int)_cal->trackwidthMm);
                    replyFn(reply, ctx); return;
                }
                if (memcmp(key, "GT", 2) == 0) {
                    _cal->turnThresholdMm = (float)v;
                    snprintf(reply, sizeof(reply), "ACK:KGT %d", (int)_cal->turnThresholdMm);
                    replyFn(reply, ctx); return;
                }
                if (memcmp(key, "GD", 2) == 0) {
                    _cal->doneTolMm = (float)v;
                    snprintf(reply, sizeof(reply), "ACK:KGD %d", (int)_cal->doneTolMm);
                    replyFn(reply, ctx); return;
                }
            }
            if (memcmp(key, "SM", 2) == 0) {
                params.minSpeedMms = v < 0 ? 0 : v;
                snprintf(reply, sizeof(reply), "ACK:KSM %d", (int)params.minSpeedMms);
                replyFn(reply, ctx);
                return;
            }
            if (memcmp(key, "SS", 2) == 0) {
                params.sTimeoutMs = clampInt(v, 50, 5000);
                snprintf(reply, sizeof(reply), "ACK:KSS %d", (int)params.sTimeoutMs);
                replyFn(reply, ctx);
                return;
            }
            if (memcmp(key, "TR", 2) == 0) {
                params.tickMs = clampInt(v, 5, 100);
                snprintf(reply, sizeof(reply), "ACK:KTR %d", (int)params.tickMs);
                replyFn(reply, ctx);
                return;
            }
            if (memcmp(key, "ER", 2) == 0) {
                params.encReportEvery = clampInt(v, 1, 20);
                snprintf(reply, sizeof(reply), "ACK:KER %d", (int)params.encReportEvery);
                replyFn(reply, ctx);
                return;
            }
            if (memcmp(key, "SD", 2) == 0) {
                params.distScale = v / 100.0f;
                snprintf(reply, sizeof(reply), "ACK:KSD %d", (int)(params.distScale * 100.0f + 0.5f));
                replyFn(reply, ctx);
                return;
            }
            if (memcmp(key, "ST", 2) == 0) {
                params.turnScale = v / 100.0f;
                snprintf(reply, sizeof(reply), "ACK:KST %d", (int)(params.turnScale * 100.0f + 0.5f));
                replyFn(reply, ctx);
                return;
            }
        }

        // Unrecognized K sub-command
        char errbuf[140];
        snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
        replyFn(errbuf, ctx);
        return;
    }

    // ── OI — OTOS init only ────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "OI", 2) == 0) {
        if (!_otos) { replyFn("ERR:OI", ctx); return; }
        _otos->begin();
        _otos->init();
        replyFn("ACK:OI", ctx);
        return;
    }

    // ── OK — calibrate IMU ─────────────────────────────────────────────────
    if (buf[0] == 'O' && len >= 2 && buf[1] == 'K') {
        if (!_otos) { replyFn("ERR:OK", ctx); return; }
        int samples = 255;
        if (len > 2) {
            int32_t args[1] = {0};
            int n = parseSignedArgs(buf + 2, args, 1);
            if (n >= 1) {
                samples = clampInt((int)args[0], 1, 255);
            }
        }
        _otos->calibrateImu((uint8_t)samples);
        replyFn("ACK:OK", ctx);
        return;
    }

    // ── OZ — reset tracking ────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "OZ", 2) == 0) {
        if (!_otos) { replyFn("ERR:OZ", ctx); return; }
        _otos->resetTracking();
        replyFn("ACK:OZ", ctx);
        return;
    }

    // ── OR — get velocity ──────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "OR", 2) == 0) {
        if (!_otos) { replyFn("ERR:OR", ctx); return; }
        int16_t vx = 0, vy = 0, vh = 0;
        _otos->getVelocityRaw(vx, vy, vh);
        int32_t vx_mms  = (int32_t)(vx * 0.153f);
        int32_t vy_mms  = (int32_t)(vy * 0.153f);
        int32_t vh_cdps = (int32_t)(vh * 6.1f);
        vx_mms  = clampInt((int)vx_mms,  -9999,  9999);
        vy_mms  = clampInt((int)vy_mms,  -9999,  9999);
        vh_cdps = clampInt((int)vh_cdps, -99999, 99999);
        char r[48];
        snprintf(r, sizeof(r), "OR%+d%+d%+d", (int)vx_mms, (int)vy_mms, (int)vh_cdps);
        replyFn(r, ctx);
        return;
    }

    // ── OP — get position ──────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "OP", 2) == 0) {
        if (!_otos) { replyFn("ERR:OP", ctx); return; }
        int16_t x = 0, y = 0, h = 0;
        _otos->getPositionRaw(x, y, h);
        int32_t x_mm   = (int32_t)(x * 0.305f);
        int32_t y_mm   = (int32_t)(y * 0.305f);
        int32_t h_cdeg = (int32_t)(h * 0.549f);
        x_mm   = clampInt((int)x_mm,   -9999,  9999);
        y_mm   = clampInt((int)y_mm,   -9999,  9999);
        h_cdeg = clampInt((int)h_cdeg, -18000, 18000);
        char r[48];
        snprintf(r, sizeof(r), "OP%+d%+d%+d", (int)x_mm, (int)y_mm, (int)h_cdeg);
        replyFn(r, ctx);
        return;
    }

    // ── OV — set position ──────────────────────────────────────────────────
    if (len > 2 && buf[0] == 'O' && buf[1] == 'V') {
        if (!_otos) { replyFn("ERR:OV", ctx); return; }
        int32_t args[3] = {0, 0, 0};
        int n = parseSignedArgs(buf + 2, args, 3);
        if (n < 3) {
            char errbuf[140];
            snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
            replyFn(errbuf, ctx);
            return;
        }
        int16_t xr = (int16_t)(args[0] / 0.305f);
        int16_t yr = (int16_t)(args[1] / 0.305f);
        int16_t hr = (int16_t)(args[2] / 0.549f);
        _otos->setPositionRaw(xr, yr, hr);
        replyFn("ACK:OV", ctx);
        return;
    }

    // ── OL — linear scalar get/set ─────────────────────────────────────────
    if (len >= 2 && buf[0] == 'O' && buf[1] == 'L') {
        if (!_otos) { char e[140]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return; }
        if (len == 2) {
            char r[16];
            snprintf(r, sizeof(r), "OL%+d", (int)_otos->getLinearScalar());
            replyFn(r, ctx);
        } else {
            int32_t args[1] = {0};
            int n = parseSignedArgs(buf + 2, args, 1);
            if (n < 1) {
                char e[140]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return;
            }
            int v = clampInt((int)args[0], -128, 127);
            _otos->setLinearScalar((int8_t)v);
            char r[32];
            snprintf(r, sizeof(r), "ACK:OL %d", v);
            replyFn(r, ctx);
        }
        return;
    }

    // ── OA — angular scalar get/set ────────────────────────────────────────
    if (len >= 2 && buf[0] == 'O' && buf[1] == 'A') {
        if (!_otos) { char e[140]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return; }
        if (len == 2) {
            char r[16];
            snprintf(r, sizeof(r), "OA%+d", (int)_otos->getAngularScalar());
            replyFn(r, ctx);
        } else {
            int32_t args[1] = {0};
            int n = parseSignedArgs(buf + 2, args, 1);
            if (n < 1) {
                char e[140]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return;
            }
            int v = clampInt((int)args[0], -128, 127);
            _otos->setAngularScalar((int8_t)v);
            char r[32];
            snprintf(r, sizeof(r), "ACK:OA %d", v);
            replyFn(r, ctx);
        }
        return;
    }

    // ── O — OTOS init + calibrate shortcut ────────────────────────────────
    if (len == 1 && buf[0] == 'O') {
        if (!_otos) { replyFn("ERR:O", ctx); return; }
        _otos->begin();
        _otos->init();
        _otos->calibrateImu(255);
        replyFn("ACK:O", ctx);
        return;
    }

    // ── LS — line sensor ───────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "LS", 2) == 0) {
        if (!_line) { replyFn("ERR:LS", ctx); return; }
        uint16_t out[4] = {0, 0, 0, 0};
        _line->readValues(out);
        char r[48];
        snprintf(r, sizeof(r), "LS%+d%+d%+d%+d",
                 (int)out[0], (int)out[1], (int)out[2], (int)out[3]);
        replyFn(r, ctx);
        return;
    }

    // ── CS — color sensor ──────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "CS", 2) == 0) {
        if (!_color) { replyFn("ERR:CS", ctx); return; }
        uint16_t cr = 0, cg = 0, cb = 0, cc = 0;
        _color->readRGBC(cr, cg, cb, cc);
        char rbuf[48];
        snprintf(rbuf, sizeof(rbuf), "CS%+d%+d%+d%+d",
                 (int)cr, (int)cg, (int)cb, (int)cc);
        replyFn(rbuf, ctx);
        return;
    }

    // ── G — go-to XY or gripper ─────────────────────────────────────────────
    if (buf[0] == 'G' && (len == 1 || buf[1] == '+' || buf[1] == '-')) {
        int32_t args[3] = {0, 0, 0};
        int n = (len > 1) ? parseSignedArgs(buf + 1, args, 3) : 0;

        if (n == 3) {
            // G+X+Y+Speed — go-to command
            float tx    = (float)args[0];
            float ty    = (float)args[1];
            float speed = fabsf((float)args[2]);
            if (speed < 1.0f) speed = 1.0f;

            _gTargetX = tx;
            _gTargetY = ty;
            _gSpeed   = speed;

            float angleRad = atan2f(ty, tx);
            float kgt = _cal ? _cal->turnThresholdMm : 50.0f;  // degrees threshold
            float angleDeg = angleRad * 57.2957795f;            // radians to degrees

            if (fabsf(angleDeg) > kgt) {
                // Pre-rotate phase: rotate in place to face target
                float turnSign = (ty >= 0.0f) ? 1.0f : -1.0f;
                _mc->startDriveClean(-turnSign * speed, turnSign * speed);
                _mc->setTarget(-turnSign * speed, turnSign * speed);
                _tgtL = -turnSign * speed;
                _tgtR =  turnSign * speed;
                // Compute how far to turn: arc length = (trackwidth/2) * |angleRad|
                float tw = _cal ? _cal->trackwidthMm : 120.0f;
                _gArcLeftMm  = -turnSign * (tw / 2.0f) * fabsf(angleRad);
                _gArcRightMm =  turnSign * (tw / 2.0f) * fabsf(angleRad);
                int32_t el, er;
                _mc->getEncoderPositions(el, er);
                _gArcStartL = (float)el;
                _gArcStartR = (float)er;
                _gPhase = GPhase::PRE_ROTATE;
                _mode = DriveMode::GO_TO;
            } else {
                // Arc phase directly (shallow angle)
                float tw = _cal ? _cal->trackwidthMm : 120.0f;
                computeArc(tx, ty, tw, _gArcLeftMm, _gArcRightMm);
                // Scale arc distances to speed ratio
                float maxArc = fmaxf(fabsf(_gArcLeftMm), fabsf(_gArcRightMm));
                float leftSpd  = (maxArc > 0.001f) ? (speed * _gArcLeftMm  / maxArc) : speed;
                float rightSpd = (maxArc > 0.001f) ? (speed * _gArcRightMm / maxArc) : speed;
                _mc->startDriveClean(leftSpd, rightSpd);
                _mc->setTarget(leftSpd, rightSpd);
                _tgtL = leftSpd;
                _tgtR = rightSpd;
                int32_t el, er;
                _mc->getEncoderPositions(el, er);
                _gArcStartL = (float)el;
                _gArcStartR = (float)er;
                _gPhase = GPhase::ARC;
                _mode = DriveMode::GO_TO;
            }

            char reply[48];
            snprintf(reply, sizeof(reply), "ACK:G %d %d %d",
                     (int)tx, (int)ty, (int)speed);
            replyFn(reply, ctx);
            return;
        }

        if (n == 1 || n == 0) {
            // Gripper backward-compat path
            if (n == 0) {
                if (!_gripper) { replyFn("ERR:G", ctx); return; }
                char r[16];
                snprintf(r, sizeof(r), "G%+d", (int)_currentGripperAngle);
                replyFn(r, ctx);
            } else {
                if (!_gripper) { replyFn("ERR:G", ctx); return; }
                int deg = clampInt((int)args[0], 0, 180);
                _gripper->setAngle((uint8_t)deg);
                _currentGripperAngle = deg;
                char r[24];
                snprintf(r, sizeof(r), "ACK:G %d", deg);
                replyFn(r, ctx);
            }
            return;
        }

        // n == 2: unrecognized
        char errbuf[140];
        snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
        replyFn(errbuf, ctx);
        return;
    }

    // ── PA — analog port read ──────────────────────────────────────────────
    if (len >= 3 && buf[0] == 'P' && buf[1] == 'A' &&
        (buf[2] == '+' || buf[2] == '-')) {
        if (!_portio) { char e[140]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return; }
        int32_t args[1] = {0};
        int n = parseSignedArgs(buf + 2, args, 1);
        if (n < 1) {
            char e[140]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return;
        }
        int val = _portio->readAnalog((uint8_t)args[0]);
        char r[32];
        snprintf(r, sizeof(r), "PA%+d%+d", (int)args[0], val);
        replyFn(r, ctx);
        return;
    }

    // ── P — digital port I/O ──────────────────────────────────────────────
    if (buf[0] == 'P' && len > 1 && (buf[1] == '+' || buf[1] == '-')) {
        if (!_portio) { char e[140]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return; }
        int32_t args[2] = {0, 0};
        int n = parseSignedArgs(buf + 1, args, 2);
        if (n < 1) {
            char e[140]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return;
        }
        char r[32];
        if (n >= 2) {
            _portio->setDigital((uint8_t)args[0], args[1] != 0);
            snprintf(r, sizeof(r), "ACK:P %d %d", (int)args[0], args[1] != 0 ? 1 : 0);
        } else {
            int val = _portio->readDigital((uint8_t)args[0]);
            snprintf(r, sizeof(r), "P%+d%+d", (int)args[0], val);
        }
        replyFn(r, ctx);
        return;
    }

    // ── Default — unrecognized command ─────────────────────────────────────
    char errbuf[140];
    snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
    replyFn(errbuf, ctx);
}

// ---------------------------------------------------------------------------
// tick — drive-mode state machine
// ---------------------------------------------------------------------------

void CommandProcessor::tick(uint32_t now_ms, ReplyFn replyFn, void* ctx)
{
    // Throttle to tickMs cadence
    if ((now_ms - _lastTickMs) < (uint32_t)params.tickMs) return;

    float dt_s    = (float)(now_ms - _lastTickMs) / 1000.0f;
    _lastTickMs   = now_ms;
    _currentTimeMs = now_ms;

    // Run motor controller and update odometry
    if (_mode != DriveMode::IDLE) {
        _mc->tick(dt_s);

        // Update odometry from encoder deltas
        int32_t encL, encR;
        _mc->getEncoderPositions(encL, encR);
        float dL = (float)(encL - _prevOdoEncL);
        float dR = (float)(encR - _prevOdoEncR);
        _prevOdoEncL = encL;
        _prevOdoEncR = encR;
        _odo->update(dL, dR, params.trackwidthMm);
    }

    // S-mode watchdog
    if (_mode == DriveMode::STREAMING) {
        if ((now_ms - _lastSMs) > (uint32_t)params.sTimeoutMs) {
            fullStop(replyFn, ctx);
            replyFn("LOG:SAFETY_STOP", ctx);
        }
    }

    // T-mode: stop when deadline reached
    if (_mode == DriveMode::TIMED && now_ms >= _tEndMs) {
        fullStop(replyFn, ctx);
        reportOdo(replyFn, ctx);
        replyFn("ACK:T+DONE", ctx);
    }

    // D-mode: stop when average encoder travel >= target, or on timeout
    if (_mode == DriveMode::DISTANCE) {
        int32_t l, r;
        _mc->getEncoderPositions(l, r);
        int32_t traveled = (abs(l - _dEncStartL) + abs(r - _dEncStartR)) / 2;
        if (traveled >= _dTargetMm || now_ms >= _dTimeoutMs) {
            fullStop(replyFn, ctx);
            reportOdo(replyFn, ctx);
            replyFn("ACK:D+DONE", ctx);
        }
    }

    // G-mode: advance go-to state machine
    if (_mode == DriveMode::GO_TO) {
        int32_t el, er;
        _mc->getEncoderPositions(el, er);
        float kgd = _cal ? _cal->doneTolMm : 5.0f;

        if (_gPhase == GPhase::PRE_ROTATE) {
            // Check if pre-rotation is complete
            float dL = fabsf((float)el - _gArcStartL);
            float dR = fabsf((float)er - _gArcStartR);
            float targetL = fabsf(_gArcLeftMm);
            float targetR = fabsf(_gArcRightMm);
            bool doneL = dL >= targetL - kgd;
            bool doneR = dR >= targetR - kgd;
            if (doneL && doneR) {
                // Advance to arc phase
                float tw = _cal ? _cal->trackwidthMm : 120.0f;
                computeArc(_gTargetX, _gTargetY, tw, _gArcLeftMm, _gArcRightMm);
                float maxArc = fmaxf(fabsf(_gArcLeftMm), fabsf(_gArcRightMm));
                float leftSpd  = (maxArc > 0.001f) ? (_gSpeed * _gArcLeftMm  / maxArc) : _gSpeed;
                float rightSpd = (maxArc > 0.001f) ? (_gSpeed * _gArcRightMm / maxArc) : _gSpeed;
                _mc->startDriveClean(leftSpd, rightSpd);
                _mc->setTarget(leftSpd, rightSpd);
                _tgtL = leftSpd;
                _tgtR = rightSpd;
                _gArcStartL = (float)el;
                _gArcStartR = (float)er;
                _gPhase = GPhase::ARC;
            }
        } else if (_gPhase == GPhase::ARC) {
            // Check if arc drive is complete
            float dL = (float)el - _gArcStartL;
            float dR = (float)er - _gArcStartR;
            bool doneL = fabsf(dL - _gArcLeftMm)  <= kgd;
            bool doneR = fabsf(dR - _gArcRightMm) <= kgd;
            if (doneL && doneR) {
                fullStop(replyFn, ctx);
                _gPhase = GPhase::IDLE;
                replyFn("G+DONE", ctx);
            }
        }
    }

    // Streaming encoder output every encReportEvery ticks
    if (_mode != DriveMode::IDLE) {
        _encTickCount++;
        if (_encTickCount >= params.encReportEvery) {
            reportEncoders(replyFn, ctx);
            if (_color) {
                uint16_t sr, sg, sb, sc;
                if (_color->pollRGBC(sr, sg, sb, sc)) {
                    char sbuf[48];
                    snprintf(sbuf, sizeof(sbuf), "CS%+d%+d%+d%+d",
                             (int)sr, (int)sg, (int)sb, (int)sc);
                    replyFn(sbuf, ctx);
                }
            }
            if (_line) {
                uint16_t lo[4] = {0, 0, 0, 0};
                _line->readValues(lo);
                char sbuf[48];
                snprintf(sbuf, sizeof(sbuf), "LS%+d%+d%+d%+d",
                         (int)lo[0], (int)lo[1], (int)lo[2], (int)lo[3]);
                replyFn(sbuf, ctx);
            }
            _encTickCount = 0;
        }
    }
}
