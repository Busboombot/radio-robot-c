# Firmware Command Reference

A lookup table for every wire-protocol command the robot firmware understands:
what it does, where it is registered, and the exact `file:line` of its parse and
handler functions. Use this to jump straight to the `G` command, the `OL`
command, etc.

> **Keep this current.** When you add, rename, or move a command, update the row
> here. The line numbers drift as files change — if a link is off by a few
> lines, search for the function name (e.g. `handleG`) rather than trusting the
> number.

## How dispatch works

A command line (from radio or serial) flows through:

1. [`CommandProcessor::process`](app/CommandProcessor.cpp#L334) — tokenizes the
   line, uppercases the verb, peels off a trailing `#id` correlation tag, and
   splits `key=value` pairs.
2. [`CommandProcessor::dispatchTable`](app/CommandProcessor.cpp#L80) — scans the
   command table for the descriptor whose `prefix` is the **longest token match**
   (so `GET VEL` beats `GET`, and `DBG LOOP RESET` beats `DBG LOOP`), then calls
   its `parseFn` and `handlerFn`.

The table itself is assembled once in
[`Robot::buildCommandTable`](robot/Robot.cpp#L1258). Each subsystem implements
the [`Commandable`](types/CommandTypes.h#L119) interface and contributes its own
commands via `getCommands()`; `buildCommandTable` concatenates them and appends
the system commands. So to find a command you (a) find which subsystem owns it,
then (b) open that subsystem's `.cpp`.

Descriptors are built with [`makeCmd`](types/CommandTypes.h#L129). The **HW**
column below is the `CMD_ACCESS_HARDWARE` flag (touches motors/sensors/GPIO/I2C);
**reply** is `ForceReply::SERIAL` for commands that always answer over the USB
serial link instead of the requesting channel.

---

## Motion — [`source/control/MotionController.cpp`](control/MotionController.cpp)

Registered in [`getCommands()`](control/MotionController.cpp#L1648).

| Cmd | Meaning | HW | Handler | Parse |
|-----|---------|----|---------|-------|
| `S` | set wheel speeds (mm/s) | | [`handleS`](control/MotionController.cpp#L881) | [`parseS`](control/MotionController.cpp#L854) |
| `T` | timed drive (ms) | | [`handleT`](control/MotionController.cpp#L954) | [`parseT`](control/MotionController.cpp#L918) |
| `D` | distance drive (mm) | | [`handleD`](control/MotionController.cpp#L1044) | [`parseD`](control/MotionController.cpp#L1010) |
| `G` | goto encoder position | | [`handleG`](control/MotionController.cpp#L1130) | [`parseG`](control/MotionController.cpp#L1098) |
| `R` | arc drive: forward speed + turn radius mm — `R <speed> <radius>` (`beginArc`, replies `OK arc`) | | [`handleR`](control/MotionController.cpp#L1191) | [`parseR`](control/MotionController.cpp#L1164) |
| `TURN` | spin in place to absolute heading, centidegrees — `TURN <cdeg> [eps=]` (`beginTurn`, replies `OK turn`) | | [`handleTURN`](control/MotionController.cpp#L1265) | [`parseTURN`](control/MotionController.cpp#L1228) |
| `VW` | velocity + angular vel (unicycle) | ✓ | [`handleVW`](control/MotionController.cpp#L1388) | [`parseVW`](control/MotionController.cpp#L1366) |
| `_VW` | raw velocity, no ramp (seed+set BVC now) | ✓ | [`handle_VW`](control/MotionController.cpp#L1571) | [`parse_VW`](control/MotionController.cpp#L1549) |
| `X` | stop immediately (`X soft` = ramp) | ✓ | [`handleX`](control/MotionController.cpp#L1617) | [`parseX`](control/MotionController.cpp#L1595) |
| `STOP` | stop with deceleration | ✓ | [`handleSTOP`](control/MotionController.cpp#L1637) | [`parseNoArgs`](control/MotionController.cpp#L1585) |

## Odometry / OTOS — [`source/control/Odometry.cpp`](control/Odometry.cpp)

Registered in [`getCommands()`](control/Odometry.cpp#L390).

| Cmd | Meaning | HW | Handler | Parse |
|-----|---------|----|---------|-------|
| `OI` | OTOS init: re-initialise sensor | ✓ | [`handleOI`](control/Odometry.cpp#L250) | [`parseOI`](control/Odometry.cpp#L173) |
| `OZ` | OTOS zero: reset position to 0,0,0 | ✓ | [`handleOZ`](control/Odometry.cpp#L265) | [`parseOZ`](control/Odometry.cpp#L180) |
| `OR` | OTOS read: one-shot position snapshot | ✓ | [`handleOR`](control/Odometry.cpp#L280) | [`parseOR`](control/Odometry.cpp#L187) |
| `OP` | OTOS position: report cached x,y,h | | [`handleOP`](control/Odometry.cpp#L305) | [`parseOP`](control/Odometry.cpp#L194) |
| `OV` | OTOS velocity: report vx,vy,omega | ✓ | [`handleOV`](control/Odometry.cpp#L329) | [`parseOV`](control/Odometry.cpp#L201) |
| `OL` | OTOS linear scalar calibration | ✓ | [`handleOL`](control/Odometry.cpp#L349) | [`parseOL`](control/Odometry.cpp#L217) |
| `OA` | OTOS angular scalar calibration | ✓ | [`handleOA`](control/Odometry.cpp#L369) | [`parseOA`](control/Odometry.cpp#L232) |

## GPIO ports — [`source/control/PortController.cpp`](control/PortController.cpp)

Registered in [`getCommands()`](control/PortController.cpp#L143).

| Cmd | Meaning | HW | Handler | Parse |
|-----|---------|----|---------|-------|
| `P` | digital pin read/write | ✓ | [`handleP`](control/PortController.cpp#L94) | [`parseP`](control/PortController.cpp#L21) |
| `PA` | analog pin read/write | ✓ | [`handlePA`](control/PortController.cpp#L115) | [`parsePA`](control/PortController.cpp#L51) |

## Servo / gripper — [`source/control/ServoController.cpp`](control/ServoController.cpp)

Registered in [`getCommands()`](control/ServoController.cpp#L76).

| Cmd | Meaning | HW | Handler | Parse |
|-----|---------|----|---------|-------|
| `GRIP` | set/query gripper angle (0–180 deg) | ✓ | [`handleGrip`](control/ServoController.cpp#L48) | [`parseGrip`](control/ServoController.cpp#L19) |

## Debug / I2C — [`source/app/DebugCommandable.cpp`](app/DebugCommandable.cpp)

Registered in [`getCommands()`](app/DebugCommandable.cpp#L444). All reply over
serial (`ForceReply::SERIAL`).

| Cmd | Meaning | HW | Handler | Parse |
|-----|---------|----|---------|-------|
| `DBG LOOP RESET` | reset loop-stats counters | | [`handleDbgLoopReset`](app/DebugCommandable.cpp#L44) | [`parseDbgLoopReset`](app/DebugCommandable.cpp#L35) |
| `DBG LOOP` | report loop timing stats | | [`handleDbgLoop`](app/DebugCommandable.cpp#L80) | [`parseDbgLoop`](app/DebugCommandable.cpp#L59) |
| `DBG I2CLOG` | dump I2C transaction log | ✓ | [`handleDbgI2clog`](app/DebugCommandable.cpp#L115) | [`parseDbgI2clog`](app/DebugCommandable.cpp#L95) |
| `DBG I2C` | report I2C bus error counts | ✓ | [`handleDbgI2c`](app/DebugCommandable.cpp#L163) | [`parseDbgI2c`](app/DebugCommandable.cpp#L143) |
| `DBG IRQGUARD` | enable/disable IRQ guard | | [`handleDbgIrqguard`](app/DebugCommandable.cpp#L236) | [`parseDbgIrqguard`](app/DebugCommandable.cpp#L219) |
| `DBG WEDGE` | run encoder wedge self-check | ✓ | [`handleDbgWedge`](app/DebugCommandable.cpp#L278) | [`parseDbgWedge`](app/DebugCommandable.cpp#L260) |
| `I2CW` | raw I2C write (addr reg data…) | ✓ | [`handleI2cw`](app/DebugCommandable.cpp#L335) | [`parseI2cw`](app/DebugCommandable.cpp#L310) |
| `I2CR` | raw I2C read (addr reg count) | ✓ | [`handleI2cr`](app/DebugCommandable.cpp#L395) | [`parseI2cr`](app/DebugCommandable.cpp#L365) |

## System — [`source/robot/Robot.cpp`](robot/Robot.cpp)

Registered directly in [`buildCommandTable`](robot/Robot.cpp#L1258); handlers are
static functions in the same file unless noted.

| Cmd | Meaning | Handler | Parse |
|-----|---------|---------|-------|
| `HELLO` | identify firmware + version | [`handleHello`](robot/Robot.cpp#L366) | [`parseHello`](robot/Robot.cpp#L360) |
| `PING` | liveness check | [`handlePing`](robot/Robot.cpp#L390) | [`parsePing`](robot/Robot.cpp#L384) |
| `ECHO` | echo tokens back | [`handleEcho`](robot/Robot.cpp#L427) | [`parseEcho`](robot/Robot.cpp#L407) |
| `ID` | report robot identity string | [`handleId`](robot/Robot.cpp#L458) | [`parseId`](robot/Robot.cpp#L452) |
| `VER` | report firmware version | [`handleVer`](robot/Robot.cpp#L509) | [`parseVer`](robot/Robot.cpp#L503) |
| `HELP` | list available commands | [`handleHelp`](robot/Robot.cpp#L530) | [`parseHelp`](robot/Robot.cpp#L524) |
| `SNAP` | emit one TLM frame on demand | [`handleSnap`](robot/Robot.cpp#L554) | [`parseSnap`](robot/Robot.cpp#L548) |
| `ZERO` | zero encoders/pose/halt-baselines | [`handleZero`](robot/Robot.cpp#L610) | [`parseZero`](robot/Robot.cpp#L569) |
| `HALT` | named stop-condition registry | [`handleHalt`](robot/Robot.cpp#L983) | [`parseHalt`](robot/Robot.cpp#L948) |
| `STREAM` | start/stop periodic TLM stream | [`handleStream`](robot/Robot.cpp#L683) | [`parseStream`](robot/Robot.cpp#L661) |
| `RF` | set radio channel | [`handleRf`](robot/Robot.cpp#L781) | [`parseRf`](robot/Robot.cpp#L764) |
| `+` | keepalive: reset watchdog | [`handleKeepalive`](robot/Robot.cpp#L914) | [`parseKeepalive`](robot/Robot.cpp#L908) |
| `GET VEL` | get velocity PID params | [`handleGetVel`](robot/Robot.cpp#L834) | [`parseGetVel`](robot/Robot.cpp#L828) |
| `GET` | get config value by key | [`handleGet`](robot/ConfigRegistry.cpp#L155) | [`parseGet`](robot/Robot.cpp#L851) |
| `SET` | set config value by key | [`handleSet`](robot/ConfigRegistry.cpp#L232) | [`parseSet`](robot/Robot.cpp#L874) |

### `GET` / `SET` keys

`GET`/`SET` don't have one handler per key — they look the key up in the config
registry table [`kRegistry[]`](robot/ConfigRegistry.cpp#L23), which maps each
friendly key name to a `RobotConfig` field by byte offset. To find or add a
tunable (e.g. `alphaYaw`, `yawRateMax`, PID gains), edit that table. Defaults
live in [`source/robot/DefaultConfig.cpp`](robot/DefaultConfig.cpp).

---

## Adding a command

1. Pick the owning subsystem (or add a new `Commandable`).
2. Write a `parseFn` (validates tokens → `ArgList`) and a `handlerFn` (does the
   work, calls `replyFn`).
3. Add a `makeCmd(...)` line to that subsystem's `getCommands()`.
4. If it's a system-level command, add it to
   [`Robot::buildCommandTable`](robot/Robot.cpp#L1258) instead — and remember
   longer prefixes must be registered so the longest-match scan reaches them.
5. Add a row to the right table above.
