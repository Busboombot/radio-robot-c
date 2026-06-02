---
id: '004'
title: SET/GET named-key config registry (replaces K* commands)
status: done
use-cases:
- SUC-003
depends-on:
- '002'
issue: protocol-v2-raw250-hard-break.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 009-004: SET/GET named-key config registry (replaces K* commands)

## Description

Implement `SET` and `GET` config commands over a static named-key registry.
These replace the 24 individual `K*` commands. The registry is a static array of
`{name, type, offsetof(RobotConfig, field)}` entries inside `CommandProcessor.cpp`.
No heap; no new module; no new dependency.

**Named-key mapping** (covers all existing `RobotConfig` fields):

| v2 key | Old K* | Type | RobotConfig field |
|---|---|---|---|
| `ml` | KML (÷1000) | float | `mmPerDegL` |
| `mr` | KMR (÷1000) | float | `mmPerDegR` |
| `kff` | KFF (÷1000) | float | `kFF` |
| `klf` | KLF (÷1000) | float | `kScaleLF` |
| `klb` | KLB (÷1000) | float | `kScaleLB` |
| `krf` | KRF (÷1000) | float | `kScaleRF` |
| `krb` | KRB (÷1000) | float | `kScaleRB` |
| `pid.kp` | KCP (÷10) | float | `ratioPidKp` |
| `pid.ki` | KCI (÷1000) | float | `ratioPidKi` |
| `pid.kd` | KCD (÷1000) | float | `ratioPidKd` |
| `pid.max` | KCC | float | `ratioPidMax` |
| `adjThr` | KAT (÷1000) | float | `kAdjThreshold` |
| `adjGain` | KAG (÷1000) | float | `kAdjGain` |
| `distScale` | KSD (÷100) | float | `distScale` |
| `turnScale` | KST (÷100) | float | `turnScale` |
| `tw` | KTW | int | `trackwidthMm` |
| `turnThr` | KGT | int | `turnThresholdMm` |
| `doneTol` | KGD | int | `doneTolMm` |
| `minSpeed` | KSM | int | `minSpeedMms` |
| `sTimeout` | KSS | int | `sTimeoutMs` |
| `tick` | KTR | int | `tickMs` |
| `tlmPeriod` | (new) | int | `tlmPeriodMs` |

**Important**: v2 passes literal decimal values — no implicit ÷10/÷100/÷1000
scaling. `SET ml=0.487` stores 0.487 directly. `GET ml` returns `0.487`.
Integer params (`tw`, `minSpeed`, etc.) are formatted as integers.

**Wire format**:
```
GET                         → CFG ml=0.487 mr=0.481 kff=0.95 … (all params, one line)
GET ml pid.kp               → CFG ml=0.487 pid.kp=2.0
SET ml=0.487 pid.kp=2.0     → OK set ml=0.487 pid.kp=2.0
SET badkey=99               → ERR badkey badkey
SET ml=0.487 bad=1          → OK set ml=0.487 (bad key skipped with ERR: emit one ERR per bad key)
```

After `SET` of PID params (`pid.kp/ki/kd/max`), call `MotorController::updatePidGains()` as the old K* handler did.

## Acceptance Criteria

- [x] `GET` (no args) returns all config params in one `CFG` line that fits in 512-byte buffer.
- [x] `GET ml pid.kp` returns only those two keys.
- [x] `SET ml=0.487 mr=0.481 tw=120` — all three keys applied; confirmed by subsequent `GET`.
- [x] `SET pid.kp=2.5` calls `MotorController::updatePidGains()`.
- [x] `SET badkey=1` → `ERR badkey badkey`; valid keys in the same command are still applied.
- [x] Integer params (`tw`, `minSpeed`, `sTimeout`, `tick`) formatted without decimal point in `GET`.
- [x] Float params formatted with 3 decimal places (e.g. `ml=0.487`).
- [x] No `K*` commands remain in the firmware.
- [x] `#id` correlation works: `GET ml #9` → `CFG ml=0.487 #9`.

## Implementation Plan

**Approach**: Add a static `ConfigEntry` table in `CommandProcessor.cpp`. The `GET`
handler iterates the table to build the response; `SET` uses `parseKV()` (from
ticket 002) to look up keys and write through via `offsetof`.

**Data structure** (stack-only, no heap):
```c
enum ConfigType { CFG_FLOAT, CFG_INT };
struct ConfigEntry {
    const char* key;
    ConfigType  type;
    size_t      offset;  // offsetof(RobotConfig, field)
};
static const ConfigEntry kRegistry[] = {
    { "ml",      CFG_FLOAT, offsetof(RobotConfig, mmPerDegL) },
    { "mr",      CFG_FLOAT, offsetof(RobotConfig, mmPerDegR) },
    // …
};
```

Float printing: use `%.3f` format (already used in newlib-nano by the old K dump;
confirm it works for the new format specifier and does not produce `NaN` or garbage).

**Files to modify**:
- `source/app/CommandProcessor.cpp` — add `kRegistry[]`, `handleGet()`, `handleSet()`
- `source/types/Config.h` — add `int32_t tlmPeriodMs` field if not present (needed for ticket 005)

**Testing**:
- `GET` → parse the response and verify all ~22 keys are present.
- `SET ml=0.500` → `GET ml` → `CFG ml=0.500`.
- `SET pid.kp=3.0` → verify PID gains updated (motor behavior change; firmware-level).
- `SET bad=1` → `ERR badkey bad`.
- Measure response length of bare `GET`; must be under 512 bytes.
