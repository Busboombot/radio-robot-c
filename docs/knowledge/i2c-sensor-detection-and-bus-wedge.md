# I2C sensor detection, bus wedge, and the day we lost to a cold-boot timing problem

**Date:** 2026-06-05  
**Sprint:** 014 (single cooperative main loop)  
**Status:** RESOLVED  

This documents the full diagnostic arc of a problem that consumed most of a day. It is
written as a narrative so future debugging sessions can recognize the pattern early.

---

## The symptom

After porting the firmware from MakeCode/TypeScript to CODAL/C++, the I2C sensors
(OTOS 0x17, line sensor 0x1A, color sensor 0x43) would intermittently fail to be detected
at boot, and sometimes the motor encoders would read `ENC 0 0` while the wheels were
visibly spinning. The `ID` command would show `caps=otos,gripper,portio` instead of
`caps=otos,line,color,gripper,portio`.

The old TypeScript firmware on the **exact same hardware** worked perfectly — all three
sensors detected and read every run.

---

## The red herring: a battery-backed wedged bus

The single biggest time-sink was this: **a wedged I2C bus persists across micro:bit
reflashes.**

When a firmware crash or hang leaves a mid-transaction I2C slave holding SDA low, the bus
is stuck. Normally you'd expect a power cycle to clear this — and on the micro:bit itself,
it does. But the Nezha motor board has a **battery backup** that keeps the peripheral side
(motors, sensors) powered even when the micro:bit USB is unplugged. So:

1. Test run X wedges the bus (sensor read mid-crash).
2. `mbdeploy deploy` flashes new firmware — micro:bit resets.
3. The Nezha and sensors **never lost power**. SDA is still held low.
4. The new firmware boots into the already-wedged bus and fails identically.
5. This looks *exactly* like a deterministic code bug. Every "new" test is contaminated.

**Rule: when debugging I2C on this robot, always do a FULL power-down** — battery switch
off (or unplug) AND USB out — wait a few seconds, then power back up. A reflash alone is
not enough. A `HELLO` coming back over serial after a reflash does NOT mean the bus is
clean; it means the micro:bit booted, which it can do even with a wedged I2C bus.

This rule wasted hours because the same fix was tried multiple times on a still-wedged bus
and appeared to fail, then "worked" when a full power cycle happened to occur for an
unrelated reason.

---

## What was actually wrong: three separate issues

### Issue 1 — `begin()` was called in the Robot constructor

The original C++ port called `_otos.begin()`, `_line.begin()`, `_color.begin()` inside the
`Robot` constructor. The Robot was constructed immediately after `uBit.init()`. On a **cold
power-on**, the line and color sensors need ~1–2 seconds to power up before their I2C
interface is ready to respond. Trying to detect them before they're ready either:
- returns a failure immediately (sensor "not found"), OR
- makes the CODAL TWIM driver wait through its error-recovery path, leaving the bus in an
  unusual state that confuses subsequent reads.

The OTOS (SparkFun) happened to be fast enough to respond even cold — explaining why
`caps=otos` but not `line,color` was the persistent failure pattern.

The fix: move `begin()` calls out of the constructor entirely. They now live in `main()`,
called straight-line before the cooperative loop starts, after a 2500 ms settle. See
[source/main.cpp](../../source/main.cpp).

### Issue 2 — Color sensor detection needed re-waking on every retry

The original MakeCode `initColor()` in `nezha.ts` wrote the wake registers (`0x81=0xCA,
`0x80=0x17`) **inside every retry iteration**, 20 retries × 50 ms apart:

```typescript
for (let i = 0; i < 20; i++) {
    colorWrite(COLOR_ALT, 0x81, 0xCA)   // re-asserted every iteration
    colorWrite(COLOR_ALT, 0x80, 0x17)
    basic.pause(50)
    let probe = colorRead(COLOR_ALT, 0xA4) + ...
    if (probe != 0) { ... return }
}
```

The C++ port initially wrote the wake registers *once* before the loop. A chip that isn't
ready on the first write never gets re-woken and fails all 20 probes.

The fix: `ColorSensor::begin()` now re-asserts the wake registers inside each retry
iteration — an exact port of the original TypeScript. See
[source/hal/ColorSensor.cpp](../../source/hal/ColorSensor.cpp).

### Issue 3 — Detection inside the cooperative loop froze it

One attempt to fix Issue 1 moved detection into the `LoopScheduler` at runtime (2.5 s into
the loop). The `begin()` calls have internal retry loops with `fiber_sleep()` calls. Running
that blocking detection inside `run_all()` froze the loop for up to 2 seconds per sensor
during detection, which meant the encoder task didn't run between attempts — and reads that
depend on the encoder task timing failed.

The fix: detection must run **before** the loop starts, straight-line in `main()`, where
blocking is fine.

---

## The architectural fix

The three issues were all caused by the same underlying problem: detection logic was buried
in constructors and loop internals, invisible and hard to reason about. The fix was to make
it explicit:

```cpp
// main() — everything visible, in order:
uBit.init();
// ... construct device objects ...
comm.begin();          // serial + radio
uBit.sleep(2500);      // let sensors power up

otos.begin();          // detect + init; sets is_initialized()
line.begin();          // retry with settle; sets is_initialized()
color.begin();         // re-wake each retry; sets is_initialized()

Robot robot(...);      // built from device objects, no raw I2C
// ... command processor, scheduler ...
sched.run_tasks();     // loop starts — sensors already initialized
```

Comment out any `begin()` line to disable that sensor. The sensor's read methods check
`is_initialized()` and early-return silently if false.

---

## The diagnostic tool that proved it

A bare-metal bring-up `main()` — constructing the HAL objects directly on `uBit.i2c` with
no Robot/CommandProcessor/LoopScheduler, reading every sensor in a plain `while (true)`
loop — was the decisive proof that the HAL and I2C code were correct. On a freshly
power-cycled bus, this test showed all sensors reading cleanly and simultaneously, including
OTOS + color + line + encoders. That killed all electrical hypotheses and pointed squarely
at initialization placement.

---

## The second time-sink: stale incremental builds

After the above fixes were in place, another several hours were lost because the running
firmware on the robot was an **old build**. Incremental `python3 build.py` was used; the
changed files compiled, but due to CMake's dependency tracking the resulting `.hex` was
byte-for-byte the same as a previous build. The `mbdeploy` output said "programmed N bytes"
(it always does — it programs whatever hex it finds), and the robot reported `fw=0.20260605.1`
both before and after.

**Rule: always use `python3 build.py --clean` before a hardware test, and always send `VER`
after flashing to confirm the live firmware version matches what you just built.** The
`dotconfig version bump` command advances the version string; without it, two different
builds look identical in `VER` output.

---

## Quick reference: the debug workflow for I2C issues on this hardware

1. **Full power-cycle** (battery off + USB out, 5 s, back on) before any diagnostic run.
   Do not trust a reflash to clear bus state.
2. Write a **bare-metal test** in `main()` — no scheduler, no Robot. Construct objects
   on `uBit.i2c` directly, call `begin()` after a settle, loop reading sensors. Eliminates
   all layers except the HAL.
3. **`--clean` build + `VER` check** after every flash to confirm the right code is running.
4. If sensors don't detect cold: add explicit prints before/after each `begin()`. The serial
   output (even raw `uBit.serial.printf`) is available before the command loop starts.
5. `DBG LOOP` (the timing dump command) reads ONLY over serial — keep the serial port
   exclusive. Two processes (VS Code Serial Monitor + a script) on one port results in
   zero bytes to both, not an error.
