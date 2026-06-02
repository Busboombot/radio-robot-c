// CommandProcessor.cpp — wire-protocol parser and dispatcher.
// All command handlers delegate to Robot public methods or component accessors.
// No hardware pointers. No config pointers. No drive state.
//
// Sprint 007, Ticket 005: thinned to Robot& + process() + static helpers.

#include "CommandProcessor.h"
#include "Robot.h"
#include "OtosSensor.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "GripperServo.h"
#include "PortIO.h"
#include "MotorController.h"
#include "Odometry.h"
#include "DriveController.h"
#include "Config.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cmath>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CommandProcessor::CommandProcessor(Robot& robot)
    : _robot(robot)
{
}

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

/**
 * Parse sign-prefixed integer arguments from a string.
 * Example: "+200-150"  -> out[0]=200, out[1]=-150, returns 2
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
        if ((unsigned char)ch < 0x21 && len == 0) continue;  // skip leading whitespace
        buf[len++] = (char)toupper((unsigned char)ch);
    }
    while (len > 0 && (unsigned char)buf[len - 1] < 0x21) --len;
    buf[len] = '\0';

    if (len == 0) return;

    RobotConfig&     cfg = _robot.config();
    MotorController& mc  = _robot.motor();  // used by K* PID setters

    // ── X or STOP — full stop ───────────────────────────────────────────────
    if ((len == 1 && buf[0] == 'X') ||
        (len == 4 && memcmp(buf, "STOP", 4) == 0)) {
        _robot.stop();
        replyFn("ACK:X", ctx);
        return;
    }

    // ── S<L><R> — streaming drive ───────────────────────────────────────────
    if (buf[0] == 'S' && len > 1 && (buf[1] == '+' || buf[1] == '-')) {
        int32_t args[2] = {0, 0};
        int n = parseSignedArgs(buf + 1, args, 2);
        if (n < 2) {
            char errbuf[264];
            snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
            replyFn(errbuf, ctx);
            return;
        }
        int minSpeed = (int)cfg.minSpeedMms;
        int leftMms  = clampMinSpeed((int)args[0], minSpeed);
        int rightMms = clampMinSpeed((int)args[1], minSpeed);

        _robot.streamDrive(leftMms, rightMms, replyFn, ctx);

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
            char errbuf[264];
            snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
            replyFn(errbuf, ctx);
            return;
        }
        int leftMms    = (int)args[0];
        int rightMms   = (int)args[1];
        int durationMs = clampInt((int)args[2], 1, 5000);

        _robot.timedDrive(leftMms, rightMms, (uint32_t)durationMs, replyFn, ctx);

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
            char errbuf[264];
            snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
            replyFn(errbuf, ctx);
            return;
        }
        int leftMms  = (int)args[0];
        int rightMms = (int)args[1];
        int targetMm = abs((int)args[2]);
        if (targetMm < 1) targetMm = 1;

        _robot.distanceDrive(leftMms, rightMms, targetMm, replyFn, ctx);

        char reply[64];
        snprintf(reply, sizeof(reply), "ACK:D %d %d %d", leftMms, rightMms, targetMm);
        replyFn(reply, ctx);
        return;
    }

    // ── ENC — query encoder positions ──────────────────────────────────────
    if (len == 3 && memcmp(buf, "ENC", 3) == 0) {
        Robot::EncoderReading enc = _robot.getEncoders();
        char rbuf[32];
        snprintf(rbuf, sizeof(rbuf), "ENC%+d%+d", (int)enc.leftMm, (int)enc.rightMm);
        replyFn(rbuf, ctx);
        return;
    }

    // ── EZ — zero encoders ─────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "EZ", 2) == 0) {
        _robot.zeroEncoders();
        replyFn("ACK:EZ", ctx);
        return;
    }

    // ── SO — query odometry pose ────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "SO", 2) == 0) {
        Robot::Pose pose = _robot.getPose();
        char rbuf[48];
        snprintf(rbuf, sizeof(rbuf), "SO%+d%+d%+d",
                 (int)pose.x_mm, (int)pose.y_mm, (int)pose.h_cdeg);
        replyFn(rbuf, ctx);
        return;
    }

    // ── SZ — zero odometry ─────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "SZ", 2) == 0) {
        _robot.zeroOdometry();
        replyFn("ACK:SZ", ctx);
        return;
    }

    // ── SI<x><y><h> — set odometry pose ────────────────────────────────────
    if (len > 2 && memcmp(buf, "SI", 2) == 0) {
        int32_t args[3] = {0, 0, 0};
        int n = parseSignedArgs(buf + 2, args, 3);
        if (n < 3) {
            char errbuf[264];
            snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
            replyFn(errbuf, ctx);
            return;
        }
        _robot.setPose(args[0], args[1], args[2]);
        char reply[64];
        snprintf(reply, sizeof(reply), "ACK:SI %d %d %d",
                 (int)args[0], (int)args[1], (int)args[2]);
        replyFn(reply, ctx);
        return;
    }

    // ── K — calibration dump or setter ─────────────────────────────────────
    if (buf[0] == 'K') {
        if (len == 1) {
            char kbuf[32];
            snprintf(kbuf, sizeof(kbuf), "K:KML:%+d", (int)(cfg.mmPerDegL * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KMR:%+d", (int)(cfg.mmPerDegR * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KFF:%+d", (int)(cfg.kFF * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KSM:%+d", (int)cfg.minSpeedMms);
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KSS:%+d", (int)cfg.sTimeoutMs);
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KTR:%+d", (int)cfg.tickMs);
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KER:%+d", (int)cfg.encReportEvery);
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KSD:%+d", (int)(cfg.distScale * 100.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KST:%+d", (int)(cfg.turnScale * 100.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KLF:%+d", (int)(cfg.kScaleLF * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KLB:%+d", (int)(cfg.kScaleLB * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KRF:%+d", (int)(cfg.kScaleRF * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KRB:%+d", (int)(cfg.kScaleRB * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KCP:%+d", (int)(cfg.ratioPidKp * 10.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KCI:%+d", (int)(cfg.ratioPidKi * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KCD:%+d", (int)(cfg.ratioPidKd * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KCC:%+d", (int)cfg.ratioPidMax);
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KAT:%+d", (int)(cfg.kAdjThreshold * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KAG:%+d", (int)(cfg.kAdjGain * 1000.0f + 0.5f));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KTW:%+d", (int)(cfg.trackwidthMm));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KGT:%+d", (int)(cfg.turnThresholdMm));
            replyFn(kbuf, ctx);
            snprintf(kbuf, sizeof(kbuf), "K:KGD:%+d", (int)(cfg.doneTolMm));
            replyFn(kbuf, ctx);
            return;
        }

        if (len >= 4) {
            char key[3] = { buf[1], buf[2], '\0' };
            int32_t args[1] = {0};
            int n = parseSignedArgs(buf + 3, args, 1);
            if (n < 1) {
                char errbuf[264];
                snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
                replyFn(errbuf, ctx);
                return;
            }
            int v = (int)args[0];
            char reply[48];

            if (memcmp(key, "ML", 2) == 0) {
                cfg.mmPerDegL = v / 1000.0f;
                snprintf(reply, sizeof(reply), "ACK:KML %d", (int)(cfg.mmPerDegL * 1000.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "MR", 2) == 0) {
                cfg.mmPerDegR = v / 1000.0f;
                snprintf(reply, sizeof(reply), "ACK:KMR %d", (int)(cfg.mmPerDegR * 1000.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "FF", 2) == 0) {
                cfg.kFF = v / 1000.0f;
                snprintf(reply, sizeof(reply), "ACK:KFF %d", (int)(cfg.kFF * 1000.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "LF", 2) == 0) {
                cfg.kScaleLF = v / 1000.0f;
                snprintf(reply, sizeof(reply), "ACK:KLF %d", (int)(cfg.kScaleLF * 1000.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "LB", 2) == 0) {
                cfg.kScaleLB = v / 1000.0f;
                snprintf(reply, sizeof(reply), "ACK:KLB %d", (int)(cfg.kScaleLB * 1000.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "RF", 2) == 0) {
                cfg.kScaleRF = v / 1000.0f;
                snprintf(reply, sizeof(reply), "ACK:KRF %d", (int)(cfg.kScaleRF * 1000.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "RB", 2) == 0) {
                cfg.kScaleRB = v / 1000.0f;
                snprintf(reply, sizeof(reply), "ACK:KRB %d", (int)(cfg.kScaleRB * 1000.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "CP", 2) == 0) {
                cfg.ratioPidKp = v / 10.0f;
                mc.updatePidGains(cfg.ratioPidKp, cfg.ratioPidKi, cfg.ratioPidKd, cfg.ratioPidMax);
                snprintf(reply, sizeof(reply), "ACK:KCP %d", (int)(cfg.ratioPidKp * 10.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "CI", 2) == 0) {
                cfg.ratioPidKi = v / 1000.0f;
                mc.updatePidGains(cfg.ratioPidKp, cfg.ratioPidKi, cfg.ratioPidKd, cfg.ratioPidMax);
                snprintf(reply, sizeof(reply), "ACK:KCI %d", (int)(cfg.ratioPidKi * 1000.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "CD", 2) == 0) {
                cfg.ratioPidKd = v / 1000.0f;
                mc.updatePidGains(cfg.ratioPidKp, cfg.ratioPidKi, cfg.ratioPidKd, cfg.ratioPidMax);
                snprintf(reply, sizeof(reply), "ACK:KCD %d", (int)(cfg.ratioPidKd * 1000.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "CC", 2) == 0) {
                cfg.ratioPidMax = (float)v;
                mc.updatePidGains(cfg.ratioPidKp, cfg.ratioPidKi, cfg.ratioPidKd, cfg.ratioPidMax);
                snprintf(reply, sizeof(reply), "ACK:KCC %d", (int)cfg.ratioPidMax);
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "AT", 2) == 0) {
                cfg.kAdjThreshold = v / 1000.0f;
                snprintf(reply, sizeof(reply), "ACK:KAT %d", (int)(cfg.kAdjThreshold * 1000.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "AG", 2) == 0) {
                cfg.kAdjGain = v / 1000.0f;
                snprintf(reply, sizeof(reply), "ACK:KAG %d", (int)(cfg.kAdjGain * 1000.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "TW", 2) == 0) {
                cfg.trackwidthMm = (float)v;
                snprintf(reply, sizeof(reply), "ACK:KTW %d", (int)cfg.trackwidthMm);
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "GT", 2) == 0) {
                cfg.turnThresholdMm = (float)v;
                snprintf(reply, sizeof(reply), "ACK:KGT %d", (int)cfg.turnThresholdMm);
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "GD", 2) == 0) {
                cfg.doneTolMm = (float)v;
                snprintf(reply, sizeof(reply), "ACK:KGD %d", (int)cfg.doneTolMm);
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "SM", 2) == 0) {
                cfg.minSpeedMms = v < 0 ? 0 : v;
                snprintf(reply, sizeof(reply), "ACK:KSM %d", (int)cfg.minSpeedMms);
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "SS", 2) == 0) {
                cfg.sTimeoutMs = clampInt(v, 50, 5000);
                snprintf(reply, sizeof(reply), "ACK:KSS %d", (int)cfg.sTimeoutMs);
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "TR", 2) == 0) {
                cfg.tickMs = clampInt(v, 5, 100);
                snprintf(reply, sizeof(reply), "ACK:KTR %d", (int)cfg.tickMs);
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "ER", 2) == 0) {
                cfg.encReportEvery = clampInt(v, 1, 20);
                snprintf(reply, sizeof(reply), "ACK:KER %d", (int)cfg.encReportEvery);
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "SD", 2) == 0) {
                cfg.distScale = v / 100.0f;
                snprintf(reply, sizeof(reply), "ACK:KSD %d", (int)(cfg.distScale * 100.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
            if (memcmp(key, "ST", 2) == 0) {
                cfg.turnScale = v / 100.0f;
                snprintf(reply, sizeof(reply), "ACK:KST %d", (int)(cfg.turnScale * 100.0f + 0.5f));
                replyFn(reply, ctx); return;
            }
        }

        char errbuf[264];
        snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
        replyFn(errbuf, ctx);
        return;
    }

    // ── OI — OTOS init only ────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "OI", 2) == 0) {
        OtosSensor* otos = _robot.otos();
        if (!otos) { replyFn("ERR:OI", ctx); return; }
        otos->begin();
        otos->init();
        replyFn("ACK:OI", ctx);
        return;
    }

    // ── OK — calibrate IMU ─────────────────────────────────────────────────
    if (buf[0] == 'O' && len >= 2 && buf[1] == 'K') {
        OtosSensor* otos = _robot.otos();
        if (!otos) { replyFn("ERR:OK", ctx); return; }
        int samples = 255;
        if (len > 2) {
            int32_t args[1] = {0};
            int n = parseSignedArgs(buf + 2, args, 1);
            if (n >= 1) {
                samples = clampInt((int)args[0], 1, 255);
            }
        }
        otos->calibrateImu((uint8_t)samples);
        replyFn("ACK:OK", ctx);
        return;
    }

    // ── OZ — reset tracking ────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "OZ", 2) == 0) {
        OtosSensor* otos = _robot.otos();
        if (!otos) { replyFn("ERR:OZ", ctx); return; }
        otos->resetTracking();
        replyFn("ACK:OZ", ctx);
        return;
    }

    // ── OR — get velocity ──────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "OR", 2) == 0) {
        OtosSensor* otos = _robot.otos();
        if (!otos) { replyFn("ERR:OR", ctx); return; }
        int16_t vx = 0, vy = 0, vh = 0;
        otos->getVelocityRaw(vx, vy, vh);
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
        OtosSensor* otos = _robot.otos();
        if (!otos) { replyFn("ERR:OP", ctx); return; }
        int16_t x = 0, y = 0, h = 0;
        otos->getPositionRaw(x, y, h);
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
        OtosSensor* otos = _robot.otos();
        if (!otos) { replyFn("ERR:OV", ctx); return; }
        int32_t args[3] = {0, 0, 0};
        int n = parseSignedArgs(buf + 2, args, 3);
        if (n < 3) {
            char errbuf[264];
            snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
            replyFn(errbuf, ctx);
            return;
        }
        int16_t xr = (int16_t)(args[0] / 0.305f);
        int16_t yr = (int16_t)(args[1] / 0.305f);
        int16_t hr = (int16_t)(args[2] / 0.549f);
        otos->setPositionRaw(xr, yr, hr);
        replyFn("ACK:OV", ctx);
        return;
    }

    // ── OL — linear scalar get/set ─────────────────────────────────────────
    if (len >= 2 && buf[0] == 'O' && buf[1] == 'L') {
        OtosSensor* otos = _robot.otos();
        if (!otos) { char e[264]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return; }
        if (len == 2) {
            char r[16];
            snprintf(r, sizeof(r), "OL%+d", (int)otos->getLinearScalar());
            replyFn(r, ctx);
        } else {
            int32_t args[1] = {0};
            int n = parseSignedArgs(buf + 2, args, 1);
            if (n < 1) {
                char e[264]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return;
            }
            int v = clampInt((int)args[0], -128, 127);
            otos->setLinearScalar((int8_t)v);
            char r[32];
            snprintf(r, sizeof(r), "ACK:OL %d", v);
            replyFn(r, ctx);
        }
        return;
    }

    // ── OA — angular scalar get/set ────────────────────────────────────────
    if (len >= 2 && buf[0] == 'O' && buf[1] == 'A') {
        OtosSensor* otos = _robot.otos();
        if (!otos) { char e[264]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return; }
        if (len == 2) {
            char r[16];
            snprintf(r, sizeof(r), "OA%+d", (int)otos->getAngularScalar());
            replyFn(r, ctx);
        } else {
            int32_t args[1] = {0};
            int n = parseSignedArgs(buf + 2, args, 1);
            if (n < 1) {
                char e[264]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return;
            }
            int v = clampInt((int)args[0], -128, 127);
            otos->setAngularScalar((int8_t)v);
            char r[32];
            snprintf(r, sizeof(r), "ACK:OA %d", v);
            replyFn(r, ctx);
        }
        return;
    }

    // ── O — OTOS init + calibrate shortcut ────────────────────────────────
    if (len == 1 && buf[0] == 'O') {
        OtosSensor* otos = _robot.otos();
        if (!otos) { replyFn("ERR:O", ctx); return; }
        otos->begin();
        otos->init();
        otos->calibrateImu(255);
        replyFn("ACK:O", ctx);
        return;
    }

    // ── LS — line sensor ───────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "LS", 2) == 0) {
        LineSensor* line = _robot.lineSensor();
        if (!line) { replyFn("ERR:LS", ctx); return; }
        uint16_t out[4] = {0, 0, 0, 0};
        line->readValues(out);
        char r[48];
        snprintf(r, sizeof(r), "LS%+d%+d%+d%+d",
                 (int)out[0], (int)out[1], (int)out[2], (int)out[3]);
        replyFn(r, ctx);
        return;
    }

    // ── CS — color sensor ──────────────────────────────────────────────────
    if (len == 2 && memcmp(buf, "CS", 2) == 0) {
        ColorSensor* color = _robot.colorSensor();
        if (!color) { replyFn("ERR:CS", ctx); return; }
        uint16_t cr = 0, cg = 0, cb = 0, cc = 0;
        color->readRGBC(cr, cg, cb, cc);
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
            float tx    = (float)args[0];
            float ty    = (float)args[1];
            float speed = fabsf((float)args[2]);
            if (speed < 1.0f) speed = 1.0f;

            _robot.goTo(tx, ty, speed, replyFn, ctx);

            char reply[48];
            snprintf(reply, sizeof(reply), "ACK:G %d %d %d",
                     (int)tx, (int)ty, (int)speed);
            replyFn(reply, ctx);
            return;
        }

        // Single-arg or bare G: gripper query/set
        if (n == 1 || n == 0) {
            GripperServo* gripper = _robot.gripper();
            if (n == 0) {
                // G with no args — query current angle
                if (!gripper) { replyFn("ERR:G", ctx); return; }
                char r[16];
                snprintf(r, sizeof(r), "G%+d", (int)_robot.gripperAngle());
                replyFn(r, ctx);
            } else {
                // G+<deg> — set gripper angle
                if (!gripper) { replyFn("ERR:G", ctx); return; }
                int deg = clampInt((int)args[0], 0, 180);
                _robot.setGripperAngle(deg);
                char r[24];
                snprintf(r, sizeof(r), "ACK:G %d", deg);
                replyFn(r, ctx);
            }
            return;
        }

        char errbuf[264];
        snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
        replyFn(errbuf, ctx);
        return;
    }

    // ── PA — analog port read ──────────────────────────────────────────────
    if (len >= 3 && buf[0] == 'P' && buf[1] == 'A' &&
        (buf[2] == '+' || buf[2] == '-')) {
        PortIO& portio = _robot.portIO();
        int32_t args[1] = {0};
        int n = parseSignedArgs(buf + 2, args, 1);
        if (n < 1) {
            char e[264]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return;
        }
        int val = portio.readAnalog((uint8_t)args[0]);
        char r[32];
        snprintf(r, sizeof(r), "PA%+d%+d", (int)args[0], val);
        replyFn(r, ctx);
        return;
    }

    // ── P — digital port I/O ──────────────────────────────────────────────
    if (buf[0] == 'P' && len > 1 && (buf[1] == '+' || buf[1] == '-')) {
        PortIO& portio = _robot.portIO();
        int32_t args[2] = {0, 0};
        int n = parseSignedArgs(buf + 1, args, 2);
        if (n < 1) {
            char e[264]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return;
        }
        char r[32];
        if (n >= 2) {
            portio.setDigital((uint8_t)args[0], args[1] != 0);
            snprintf(r, sizeof(r), "ACK:P %d %d", (int)args[0], args[1] != 0 ? 1 : 0);
        } else {
            int val = portio.readDigital((uint8_t)args[0]);
            snprintf(r, sizeof(r), "P%+d%+d", (int)args[0], val);
        }
        replyFn(r, ctx);
        return;
    }

    // ── Default — unrecognized command ─────────────────────────────────────
    char errbuf[264];
    snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
    replyFn(errbuf, ctx);
}

