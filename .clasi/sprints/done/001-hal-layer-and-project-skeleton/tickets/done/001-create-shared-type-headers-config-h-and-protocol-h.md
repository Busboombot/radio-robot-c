---
id: '001'
title: Create shared type headers Config.h and Protocol.h
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on: []
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Create shared type headers Config.h and Protocol.h

## Description

Create the two foundational type headers that every other module in the
firmware depends on. These are header-only files with no corresponding `.cpp`.
They live in `source/types/` and must compile cleanly with no warnings.

No other sprint 001 ticket can start until this ticket is done.

## Files to Create

- `source/types/Config.h`
- `source/types/Protocol.h`

## Acceptance Criteria

- [x] `source/types/Config.h` exists and defines `CalibParams`, `MotorGains`, and `DriveMode`
- [x] `source/types/Protocol.h` exists and defines all `PROTO_*` constants, `ReplyFn`, and `ReplyCtx`
- [x] Both headers have include guards (`#pragma once`)
- [x] `Config.h` includes only `<stdint.h>` and `<stddef.h>` — no CODAL headers
- [x] `Protocol.h` includes only `<stdint.h>` and `<stddef.h>` — no CODAL headers
- [ ] `python build.py` compiles successfully (or the build container confirms no type errors)

## Implementation Plan

### `source/types/Config.h`

```cpp
#pragma once
#include <stdint.h>

struct CalibParams {
    float mmPerDegL;       // encoder deg → mm, left wheel  (default 0.487)
    float mmPerDegR;       // encoder deg → mm, right wheel (default 0.481)
    float kFF;             // feed-forward gain              (default 0.15)
    float kScaleLF;        // left-forward scale             (default 1.0)
    float kScaleLB;        // left-backward scale            (default 1.0)
    float kScaleRF;        // right-forward scale            (default 1.0)
    float kScaleRB;        // right-backward scale           (default 1.0)
    float kAdjThreshold;   // slower-wheel adj threshold     (default 0.5)
    float kAdjGain;        // slower-wheel adj gain          (default 0.05)
    float trackwidthMm;    // axle width mm                  (default 120.0)
    float ratioPidKp;      // ratio PID Kp                   (default 300.0)
    float ratioPidKi;      // ratio PID Ki                   (default 0.0)
    float ratioPidKd;      // ratio PID Kd                   (default 0.0)
    float ratioPidMax;     // ratio PID output clamp         (default 30.0)
    float turnThresholdMm; // threshold to detect a turn
    float doneTolMm;       // completion tolerance for T/D commands
};

// Sensible defaults matching nezha.ts calibration constants.
inline CalibParams defaultCalibParams() {
    CalibParams p{};
    p.mmPerDegL       = 0.487f;
    p.mmPerDegR       = 0.481f;
    p.kFF             = 0.15f;
    p.kScaleLF        = 1.0f;
    p.kScaleLB        = 1.0f;
    p.kScaleRF        = 1.0f;
    p.kScaleRB        = 1.0f;
    p.kAdjThreshold   = 0.5f;
    p.kAdjGain        = 0.05f;
    p.trackwidthMm    = 120.0f;
    p.ratioPidKp      = 300.0f;
    p.ratioPidKi      = 0.0f;
    p.ratioPidKd      = 0.0f;
    p.ratioPidMax     = 30.0f;
    p.turnThresholdMm = 5.0f;
    p.doneTolMm       = 3.0f;
    return p;
}

struct MotorGains {
    float kp;
    float ki;
    float kff;
};

enum class DriveMode : uint8_t {
    IDLE      = 0,
    STREAMING = 1,
    TIMED     = 2,
    DISTANCE  = 3,
    GO_TO     = 4
};
```

### `source/types/Protocol.h`

```cpp
#pragma once
#include <stdint.h>

// ── Inbound command prefixes ─────────────────────────────────────────
constexpr const char* PROTO_CMD_HELLO    = "HELLO";

// ── Outbound reply prefixes ──────────────────────────────────────────
constexpr const char* PROTO_REPLY_DEVICE = "DEVICE:";
constexpr const char* PROTO_REPLY_LOG    = "LOG:";
constexpr const char* PROTO_REPLY_OK     = "OK";
constexpr const char* PROTO_REPLY_ERR    = "ERR:";

// ── Reply callback type ──────────────────────────────────────────────
// Avoids std::function; no heap. ctx carries routing info.
using ReplyFn = void(*)(const char* msg, void* ctx);

struct ReplyCtx {
    bool viaSerial;   // send over serial
    bool viaRadio;    // send over radio
    bool relay;       // if viaRadio: prepend '<' (relay mode)
};
```

## Testing

Hardware-in-the-loop only (no off-device test runner for CODAL).

- **Verification**: `python build.py` must complete without errors. The headers
  are included by all subsequent tickets; if they compile, the headers are correct.
- **No new tests**: Type-only headers have no runtime behavior to test in isolation.
