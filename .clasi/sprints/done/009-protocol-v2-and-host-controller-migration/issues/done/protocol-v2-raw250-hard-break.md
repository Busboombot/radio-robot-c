---
status: done
sprint: 009
tickets:
- '001'
- '002'
- '003'
- '004'
- '005'
- '006'
- '007'
- 008
- 009
---

# Protocol v2: RAW250 Text Redesign (Hard Break) + Host Controller Migration

## Context

The robot↔host command/telemetry protocol was designed for the legacy
19-char MakeCode radio limit. The RadioRelay now runs in **RAW250 mode**, and
the **transport is already done**: `source/hal/Radio.cpp` implements the RAW250
fragment framing (`[SEQ][FLAGS][LEN]`, 247-byte MTU, auto fragment + reassemble)
and `Robot::run()` (`source/app/Robot.cpp:60`) already polls both serial and
radio through the same `CommandProcessor`, replying on whichever channel a
command arrived on. So this work is a **protocol-vocabulary redesign**, not a
transport change (one buffer-sizing change excepted — see Constraints).

We are taking a **hard break** — no backward compatibility with the legacy
terse protocol. There is a lot of flux right now, so a clean break is cheaper
than dual-parsing.

**Decisions locked with the stakeholder:**
- **Hard break** — drop the legacy single-letter packed commands and the
  `DEVICE:`/`HELLO` format. `proto=2` is a clean slate.
- **Decimals only where required.** Use decimal text for values that are
  genuinely fractional (calibration gains, mm/deg, PID terms). **Distances stay
  integer millimeters.** There are **no implicit ×10 (or other) scaling
  multipliers** anywhere in v2 — every value is its literal magnitude, so no
  encode/decode scaling logic is needed.
- **Host controller: copy over wholesale.** The current host controller lives at
  `/Volumes/Proj/proj/league-projects/scratch/radio-robot/robot_radio`. We are
  moving development into this repo and doing a full rewrite, so copy that
  package in as the starting point and adapt it to speak protocol v2.

## Pain points v2 fixes

1. **One `K` command per constant**; a `K` dump emits ~24 separate lines.
2. **Telemetry is scattered** — `CommandProcessor::tick()` emits `ENC…`, `CS…`,
   `LS…` as separate, uncorrelated, untimestamped lines every cycle.
3. **Terse sign-packed numbers** (`S+200-150`) are hard to read/debug.
4. **`G` is overloaded** (go-to XY vs gripper angle).
5. **Inconsistent reply tags** (`ACK:`, `ERR:`, `LOG:`, `+DONE`, bare data).
6. **`process()` upper-cases the whole line**, which is incompatible with
   `key=value` names and echo payloads.
7. **No echo / ping / version / capability** beyond `HELLO`.

## Proposed protocol v2

Text, line-oriented (one message = one `\n`-terminated line), whitespace-
delimited tokens plus `key=value`. Verb-only upper-casing (preserve case in
keys/values/echo payloads).

### Response taxonomy (leading tag)
| Tag | Meaning |
|---|---|
| `OK`  | command accepted / result |
| `ERR` | rejected (`ERR <code> <detail>`, e.g. `badarg`,`badkey`,`nodev`,`range`) |
| `EVT` | async event (`EVT done …`, `EVT safety_stop`) |
| `TLM` | telemetry frame |
| `CFG` | config dump |
| `ID`  | identity / capabilities |

Optional request correlation: a command may carry a trailing `#<id>`; the
response echoes it (`drive 200 150 #7` → `OK drive l=200 r=150 #7`).

### Liveness & identity
- `PING` → `OK pong t=<robot_ms>` (also the clock-sync probe — see Time synchronization)
- `ECHO <text…>` → `OK echo <text…>` (round-trip / large-message test — the
  intended way to verify fragmentation+reassembly both directions)
- `ID` → `ID model=Nezha2 name=GUTOV serial=… fw=<ver> proto=2 caps=otos,line,color,gripper,portio`
- `VER` → firmware + protocol version; `HELP` → compact command index

### Config: `SET` / `GET` (replaces per-constant `K`)
```
SET ml=0.487 mr=0.481 tw=120 sTimeout=200 pid.kp=2.0 pid.ki=0.05
  → OK set ml=0.487 mr=0.481 tw=120 sTimeout=200 pid.kp=2.0 pid.ki=0.05
GET            → CFG ml=0.487 mr=0.481 distScale=0.94 turnScale=1.07 minSpeed=50 tick=20 …
GET ml pid.kp  → CFG ml=0.487 pid.kp=2.0     (subset)
```
Named keys replace opaque `KCP`/`KAT`/etc. Decimals where fractional; integers
(e.g. `tw`, `minSpeed`, `sTimeout`) stay integer. `SET` applies valid keys and
reports rejected ones (`ERR badkey foo`).

### Telemetry: one combined frame (replaces scattered `ENC`/`CS`/`LS`/`SO`)
```
TLM t=12345 mode=S enc=1024,1019 pose=350,-12,1780 vel=200,0,15 line=120,340,330,118 color=21,30,18,80
```
- `t` robot ms (correlate snapshots / detect drops); only fields for present
  sensors are included. `pose` heading in centi-degrees, integer mm positions.
- `STREAM <ms>` set telemetry period (0=off); `STREAM fields=enc,pose,line`
  subscribe to a subset; `SNAP` one-shot immediate `TLM` frame.

### Time synchronization

The robot clock is free-running `uBit.systemTime()` (ms since boot). We do **not**
make the two clocks read the same value; we keep the robot clock monotonic and
estimate the offset (and optionally skew) **on the host**, translating robot
timestamps into host time.

- **`PING` is the time probe.** It returns the robot clock: `OK pong t=<robot_ms>`.
  Host records `T0` before send and `T1` on receipt; with `t_r` from the reply and
  assuming ~symmetric link delay, the robot's clock corresponds to host time
  `(T0+T1)/2` at stamp `t_r`, so `offset ≈ (T0+T1)/2 − t_r`.
- **Min-RTT filtering (NTP trick).** Fire several `PING`s and keep the sample with
  the smallest round-trip — it has the least relay/queuing jitter. Accuracy is
  bounded by ~½ the minimum RTT (a few ms to low tens of ms over the half-duplex
  relay), comfortably finer than the 40–80 ms sensor periods.
- **Skew (optional).** Linear-regress several `PING` samples spread over time to fit
  `host ≈ a·t_robot + b`; micro:bit crystal drift is ~tens of ppm (a few ms/min).
  Re-PING every ~30–60 s to keep the offset tight.
- **Stamp the measurement, not the transmission.** A `TLM`'s `t=` MUST be the robot
  clock *when the sensors were sampled*, not when the line was sent — otherwise the
  translated time carries the variable send latency. (Firmware discipline; affects
  the multi-rate scheduler in the controller-rewrite issue.)
- **Why host-side offset, not setting the robot clock:** keeps robot time monotonic
  (a jump would corrupt odometry `dt`), needs zero extra firmware (`PING` already
  returns `t`), and is robust to the relay's asymmetric, jittery latency.
- **Optional later:** `SETTIME <host_ms>` that stores an additive robot-side offset
  for epoch alignment. Start with host-side offset only.

### Motion (kept semantics, spaced + cleaned up)
- `S 200 150` streaming (watchdog) · `T 200 150 1000` timed · `D 200 200 300`
  distance · `G 300 0 200` go-to · `STOP`
- **Gripper de-overloaded:** `GRIP 90` / `GRIP` → `OK grip deg=90`. `G` is now
  unambiguously go-to.

### Misc
- `ZERO enc pose` umbrella (replaces separate `EZ`+`SZ`).
- OTOS/port commands carry over with the same word+`key=value` treatment.

## Relationship to the controller-rewrite issue

`controller-rewrite-hybrid-drive-to-pose-base.md` defines new drive commands
(`BV`, `DP`, `DT`, `DL`, `PC`, `WC`, `QF/QE/QO/QV`, velocity/fusion tunables)
but specifies them in the **legacy ≤19-char sign-prefix framing**. Protocol v2
is the **wire format those commands should be expressed in** (spaced tokens,
`key=value`, `OK/ERR/EVT/TLM` taxonomy, decimals where required). When both are
scheduled, v2 should land first (or jointly) so the controller-rewrite commands
are authored directly in the v2 format rather than the terse one. Reconcile the
two command tables during sprint planning.

## Constraints / gotchas

- **Buffer ceiling (the one real non-parser code change).** `Radio.h`
  `REASM_MAX = 256` and `Robot.h` `_buf[256]` cap a single message at 255
  chars. A **full `GET` dump (~24 params ≈ 290 bytes) will not fit in one
  line.** Raise `REASM_MAX` / `_buf` / TX buffer to ~512 (relay reassembles up
  to 1024; transport already fragments). Same applies to a large `SET`.
- **Single-message buffering.** `Radio::poll` keeps only the latest completed
  message; keep telemetry periods sane (≥20 ms).
- **Case-folding.** Tokenize first, upper-case only the verb.
- **Float printf** must be enabled in the newlib-nano build (already used by the
  `K` dump).

## Scope of work (natural sprint/ticket boundaries)

1. **Buffers & framing** — raise `REASM_MAX`/`_buf`/TX to ~512; confirm
   `codal.json` `MICROBIT_RADIO_MAX_PACKET_SIZE=250`.
2. **Parser core** — tokenizer (verb + positional + `key=value`), verb-only
   upper-casing, `#id` correlation, `OK/ERR/EVT` taxonomy. Hard-remove legacy
   packed parsing.
3. **`PING`/`ECHO`/`ID`/`VER`/`HELP`.**
4. **`SET`/`GET`** over existing `Params`/`CalibParams` (named registry).
5. **Unified `TLM` + `STREAM`/`SNAP`** (refactor `tick()` streaming). Stamp `TLM
   t=` at sensor-sample time, not send time.
5a. **Time sync** — host-side offset/skew estimator over `PING` (min-RTT
   filtering), translating robot `t` → host time. Robot side is just the existing
   `PING t=`; optional `SETTIME` deferred.
6. **Motion verbs + `GRIP` de-overload + `ZERO` umbrella.**
7. **Host controller migration** — copy
   `/Volumes/Proj/proj/league-projects/scratch/radio-robot/robot_radio` into
   this repo and adapt its serial/radio protocol layer to v2 (it holds
   `controllers/`, `nav/`, `kinematics/`, `path/`, `sensors/`, `robot/`,
   `io/`, `config/`). Update its `protocol.py`/`nezha.py` equivalents to the new
   tags and `key=value` config.
8. **Docs** — a `protocol-v2` spec doc mirroring the relay's
   `radio-relay-protocol.md`.

## Verification

- `ECHO` of a ~200-byte payload round-trips intact over the relay (proves
  fragmentation+reassembly both directions).
- `GET` returns the full config in one (now larger-buffer) line; `SET` of
  several keys at once is reflected by a subsequent `GET`.
- A single `TLM` frame carries enc+pose+sensors with a timestamp; `STREAM`
  controls cadence.
- Host controller drives the robot end-to-end over the relay using v2.
- Clock-sync: after a `PING` burst, the host's robot→host time translation aligns a
  known robot event (e.g. an `EVT done`) with the host clock to within ~½ the
  measured minimum RTT; offset stays stable across a multi-minute run.
