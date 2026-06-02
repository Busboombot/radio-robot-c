---
status: pending
---

# Full Vendor I2C Coverage in the Nezha HAL

Split from the Nezha vendor-I2C-coverage plan; sibling of (and follows)
[[nezha-chip-velocity-readspeed-0x47]] (the velocity command `0x47` is covered
there, not here).

## Context

The advisory vendor driver `pxt-nezha2` (`main.ts`) is the source of truth for
the Nezha2 chip's I2C command set, but it currently lives **only** in the scratch
repo — this repo's `vendor/` dir has no copy. Our HAL
(`source/hal/NezhaV2.{h,cpp}`) wraps only drive (`0x60`), stop (`0x5F`), and
encoder read (`0x46`); several vendor commands are unwrapped. We want **every**
vendor I2C command represented so we can use the chip at full capacity, even
commands our own control loops may not use.

## Vendor I2C command surface (frame `[0xFF,0xF9,motor,p3,reg,p5,p6,p7]`, addr `0x10`, M1=right / M2=left)

| reg | dir | purpose | encoding | wrapped today |
|---|---|---|---|---|
| `0x60` | W | continuous PWM drive | dir(1=CW,2=CCW)+speed 0–100 | ✅ `setPwm` |
| `0x5F` | W | stop / brake | zeros | ✅ |
| `0x46` | RW | encoder angle | int32 LE, 0.1°/LSB | ✅ `readEncoder` |
| `0x47` | RW | velocity (readSpeed) | uint16 LE, `floor(raw/3.6)*0.01` laps/s | → [[nezha-chip-velocity-readspeed-0x47]] |
| `0x70` | W | timed move | dir + value + mode(1=turns,2=deg,3=sec) | ❌ |
| `0x5D` | W | move to absolute angle | angle 0–359 + mode(1=shortest,2=CW,3=CCW); **`delayMs(4)` after is BUG-critical, no task interleave** | ❌ |
| `0x1D` | W | reset / home (zero encoder) | — | ❌ |
| `0x77` | W | global servo speed | speed×9 → 0–900 | ❌ |
| `0x88` | RW | firmware version | 3 bytes major.minor.patch | ❌ |

## Scope

**Task 1 — Vendor the advisory reference.** Copy `pxt-nezha2/main.ts` (and
`test.ts`) into `radio-robot-c/vendor/pxt-nezha2/` with a short `README` noting it
is **advisory** / the authoritative I2C source and is **not compiled**. Makes the
coverage gap auditable in-repo. Source:
`/Volumes/Proj/proj/league-projects/scratch/radio-robot/vendor/pxt-nezha2/main.ts`.

**Task 2 — Wrap the remaining commands** in `source/hal/NezhaV2.{h,cpp}`
(`0x47` is handled by the velocity issue):
- `0x70` timed move (turns/deg/sec), `0x5D` move-to-absolute-angle, `0x1D`
  reset/home, `0x77` global servo speed, `0x88` read version.
- Preserve frame layout, motor-ID mapping, direction conventions, and the
  **BUG-critical `0x5D` post-write delay with no task interleave** (quote the
  vendor comment). Keep the 4 ms read padding for reads.
- Note in code which commands overlap our own control (`0x70`/`0x5D`) — wrapped
  for completeness, not necessarily used by our control stack.

**Coverage checklist.** Maintain a `vendor reg → HAL method` table, kept green, so
future audits confirm nothing is missing.

## Verification

- Unit tests asserting exact frame bytes for each new command.
- `0x88` returns a plausible version string on hardware.
- Coverage checklist shows every vendor I2C register has a HAL wrapper.
