# Encoders read zero when color + OTOS are both read — a FIRMWARE bug

**Status:** open. **This is NOT an electrical / signal-integrity / "marginal bus" problem.**
An earlier version of this note concluded "electrical" — that was wrong (see below).

## Symptom

Encoder position + velocity read 0 while the motors still run (wheels spin, often fast
because the velocity loop sees 0 and saturates PWM). On the wire: `ENC 0 0  VEL 0 0`. It
happens once the firmware **reads** both the color sensor (0x43) and the OTOS (0x17) — and
even the boot-time *detection* read of one while the other is present can trigger it,
leaving the bus such that the motor encoder read (0x10) also returns 0.

## Why it is a FIRMWARE bug, not hardware (decisive evidence)

- **Commercial plug-and-play hardware** (micro:bit V2 + Nezha motor board + SparkFun OTOS +
  PlanetX color/line). Nothing exotic.
- **The old MakeCode/pxt firmware reads ALL sensors — OTOS + color + line — flawlessly on the
  exact same hardware.** Source: `/Volumes/Proj/proj/league-projects/scratch/radio-robot/src`.
- With **our** firmware, all three sensors physically present but **sensor reads OFF** →
  encoders count perfectly. So *presence* is fine; our *reads* are the trigger.

So the defect is in **how our CODAL firmware performs I2C**, not in the bus. It is fully
solvable — the old code is the proof and the reference.

## Bisection (our firmware)

Using the `run_all` loop + `DBG LOOP <x> <state>` task toggles (LoopScheduler):
- All three present, **read none** → encoders count. ✅
- Enable the **OTOS read** (with color present) → wedged. ❌
- Enable the **color read** (with OTOS present) → wedged. ❌ (symmetric)
- Either chip alone, fully read → fine.

So reading *either* chip while *both* are present wedges subsequent reads; reading neither is
fine. Boot detection reads both, so it can wedge from boot (intermittently), which also makes
OTOS/color fail to appear in `ID … caps=…`.

## Tried and did NOT fix it (so these are not the cause)

- Matching the upstream PlanetX single-byte color read protocol.
- Setting the I2C bus to 100 kHz (`uBit.i2c.setFrequency(100000)`) — note: not confirmed the
  call actually changed the bus clock.
- Switching OTOS + color register reads to repeated-start (write with `repeated=true`, no STOP
  between write-reg and read).

## Where to look next (firmware)

The difference is our CODAL `MicroBitI2C` usage vs the MakeCode/pxt runtime's I2C path. Suspects:
- CODAL nRF52 **TWIM** handling of back-to-back transactions across multiple device addresses
  (motor 0x10 → otos 0x17 → color 0x43) — a driver-level hang/lockup that needs a re-init or
  different sequencing, which the slower pxt runtime never trips.
- Which I2C **instance/bus** we use (`uBit.i2c`) and how it's configured vs the old code.
- Compare the exact old-code I2C call sequence/ordering and replicate it.

## Recovery while wedged

A full power-down (battery + USB) clears the wedged state; a micro:bit-only reset/reflash does
not (the battery keeps the peripheral side powered). This is a *recovery* note, not a diagnosis.

## Tooling

`LoopScheduler::run_all()` runs every task explicitly with per-task `run`/`run_always`/`run_once`
flags and timing (`runs`/`totalTimeUs`); `DBG LOOP <x> <0|1>` toggles a task at runtime and
`DBG LOOP` lists them. Use it to bisect which read triggers the fault without rebuilding.
