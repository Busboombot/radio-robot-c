---
sprint: "001"
revision: 1
---

# Architecture Update — Sprint 001: HAL Layer and Project Skeleton

See `.clasi/sprints/001-hal-layer-and-project-skeleton/architecture-update.md`
for the full architecture update document with all module interfaces, diagrams,
design rationale, and open questions.

## Summary

This sprint introduces the entire `source/` directory tree from a placeholder
`main.cpp`. The modules introduced are:

| Directory | Files |
|---|---|
| `source/types/` | `Config.h`, `Protocol.h` |
| `source/hal/` | `NezhaV2`, `OtosSensor`, `LineSensor`, `ColorSensor`, `GripperServo`, `PortIO`, `SerialPort`, `Radio` |
| `source/app/` | `Announcer`, `Robot` |
| `source/` | `main.cpp` (replacement) |

No control, navigation, or full CommandProcessor code is included.

## Key Design Constraints Established

| Constraint | Value |
|---|---|
| `MicroBit uBit` | First member of `Robot`; controls init order |
| I2C motor address | 0x10 (Nezha V2); LEFT=M2, RIGHT=M1; LEFT_FWD=+1, RIGHT_FWD=−1 |
| OTOS address | 0x17; register map documented in architecture-update.md |
| Tick period | 20 ms; `uBit.sleep(20)` not busy-wait |
| Optional sensors | Nullable members; robot boots without any sensor connected |
| No heap allocation | All buffers and instances are stack or static |
| Radio ring buffer | 4 slots × 64 bytes; ISR-driven write; singleton `_instance` pointer |
| Relay protocol | Inbound `>` stripped; outbound relay prepends `<` |

## Architecture Review

Status: **APPROVED** — no significant issues. Three open questions flagged
for programmer agents (Nezha V2 I2C byte layout, CODAL serial ASYNC constant,
servo pin API). None are architectural blockers.
