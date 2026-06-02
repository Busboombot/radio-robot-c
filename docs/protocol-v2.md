# Protocol v2 Wire Specification

Version 2 of the Nezha firmware command/telemetry protocol.
Hard break from v1 — no backward compatibility.

---

## 1. Overview

Protocol v2 is a line-oriented text protocol: one message equals one
`\n`-terminated line.  Tokens are whitespace-delimited.  `key=value`
pairs carry named parameters.  Only the first token (the verb) is
upper-cased; all remaining tokens, keys, and values preserve the case
as sent.  The protocol identifier is `proto=2`.

v1 commands (`K*`, `ENC`, `SO`, `SSE`, `SSO`, `SSC`, `SSL`, `HELLO`,
`DEVICE:`, and packed sign-prefix motion verbs) are removed.  Any
unrecognised verb returns `ERR unknown`.

Transport: the RadioRelay operates in RAW250 mode (247-byte MTU,
`[SEQ][FLAGS][LEN]` fragment framing).  Fragmentation and reassembly
are handled transparently by the firmware HAL (`Radio.cpp`) and the
relay.  The serial path (115 200 baud) accepts full lines directly.
The application layer sees only complete, NUL-terminated lines — it is
not aware of transport framing.

---

## 2. Grammar

```
message   ::= verb [token…] ['#' corr_id] '\n'
verb      ::= UPPER-CASE-WORD         ; upper-cased by firmware on receive
token     ::= positional | key_value
positional ::= non-whitespace-string  ; no '=' character (or '=' not at start)
key_value  ::= key '=' value
key       ::= non-whitespace-string   ; no '='
value     ::= non-whitespace-string   ; may be empty
corr_id   ::= one-or-more decimal digits
```

Rules:
- Leading and trailing whitespace is stripped from the line.
- Tokens are split on any run of space or tab characters.
- The verb token is upper-cased in place; all other tokens preserve case.
- A trailing `#<digits>` token (decimal digits only, no letters) is
  extracted as the *correlation id* and is not counted as a positional
  argument.  The firmware echoes it in every synchronous response for
  that command.  Async `EVT done T/D/G` and `EVT safety_stop` events
  also echo the `#id` when the originating drive command carried one;
  bare events (no originating id) carry no `#id`.
- A `key=value` token with an empty key (starts with `=`) is rejected
  with `ERR badarg missing key`.
- A `key=value` token with an empty value (ends with `=`) is valid; the
  value is the empty string.

---

## 3. Response Taxonomy

Every response line begins with one of six tags.

| Tag   | Meaning                          | Example                                      |
|-------|----------------------------------|----------------------------------------------|
| `OK`  | Command accepted / result        | `OK pong t=12345`                            |
| `ERR` | Rejected                         | `ERR badarg missing key`                     |
| `EVT` | Async event (unsolicited)        | `EVT done T`                                 |
| `TLM` | Telemetry frame                  | `TLM t=12345 mode=S enc=1024,1019`           |
| `CFG` | Config dump (response to GET)    | `CFG ml=0.487 mr=0.481 tw=120`               |
| `ID`  | Identity / capabilities          | `ID model=Nezha2 name=GUTOV serial=… proto=2` |

**OK format:**

```
OK <verb> [<body>] [#<corr_id>]
```

**ERR format:**

```
ERR <code> [<detail>] [#<corr_id>]
```

**EVT format:**

```
EVT <name> [<body>] [#<corr_id>]
```

`#<corr_id>` is present on `EVT done T/D/G` and `EVT safety_stop` only when
the originating T/D/G command carried a `#id`.  Uncorrelated drives produce
bare events with no `#id`.

---

## 4. Error Codes

| Code      | Meaning                                                      |
|-----------|--------------------------------------------------------------|
| `unknown` | Verb not recognised; detail is the verb that was sent        |
| `badarg`  | Wrong number or type of positional arguments                 |
| `badkey`  | Unknown `key=value` key; detail is the offending key name    |
| `nodev`   | Hardware device not present; detail is the command verb      |
| `range`   | Numeric argument outside the allowed range; detail is param  |

---

## 5. `#id` Correlation

A command may carry a trailing `#<digits>` token:

```
PING #7
T 200 200 1000 #42
GET ml pid.kp #9
```

The firmware strips the `#id` token from the argument list and echoes
it in every *synchronous* response for that command:

```
OK pong t=12345 #7
OK drive l=200 r=200 ms=1000 #42
CFG ml=0.487 pid.kp=300.000 #9
```

Rules:
- The id must consist of decimal digits only (no letters or other chars).
- If the last token begins with `#` but contains non-digit characters it
  is treated as a positional argument, not a correlation id.
- `EVT done T/D/G` and `EVT safety_stop` echo the `#id` of the originating
  T, D, or G command when that command carried one.  S-mode watchdog
  (`EVT safety_stop`) echoes the `#id` of the S command that established
  the session, if any.  Commands with no `#id` produce bare events.
- Multiple `ERR` lines from a single `SET` command (one per bad key) each
  carry the correlation id.

---

## 6. Liveness / Identity

### PING

```
PING [#id]
→ OK pong t=<robot_ms> [#id]
```

`t` is the robot clock in milliseconds since boot (`uBit.systemTime()`).
This is the time-synchronisation probe — see §9.

Example:

```
PING
OK pong t=12345

PING #3
OK pong t=12347 #3
```

### ECHO

```
ECHO <payload…> [#id]
→ OK echo <payload…> [#id]
```

The payload is everything after the `ECHO` token, with original
whitespace and case preserved (extracted from the raw line).  The
trailing `#id` token, if present, is stripped from the echoed payload.

Used to verify fragmentation+reassembly for large messages over the
relay; the payload may be up to ~490 bytes (512-byte buffer, minus
`ECHO ` prefix and `OK echo ` reply prefix).

Example:

```
ECHO hello world
OK echo hello world

ECHO the-quick-brown-fox #1
OK echo the-quick-brown-fox #1
```

### ID

```
ID [#id]
→ ID model=Nezha2 name=<name> serial=<serial> fw=<ver> proto=2 caps=<caps> [#id]
```

The response tag is `ID`, not `OK`.  Fields:

| Field    | Value                                                             |
|----------|-------------------------------------------------------------------|
| `model`  | Always `Nezha2`                                                   |
| `name`   | micro:bit friendly name (5-character CODAL name)                  |
| `serial` | Hardware serial number (decimal `uint32_t`)                        |
| `fw`     | Firmware version string (e.g. `0.20260602.6`)                     |
| `proto`  | Protocol version; always `2` for v2 firmware                      |
| `caps`   | Comma-separated list of detected subsystems (see below)           |

`caps` values: `otos`, `line`, `color`, `gripper`, `portio`.  `portio`
is always present.  Others are included only when hardware was detected
at boot.

Example:

```
ID
ID model=Nezha2 name=GUTOV serial=1234567 fw=0.20260602.6 proto=2 caps=otos,line,color,gripper,portio
```

### VER

```
VER [#id]
→ OK ver fw=<ver> proto=2 [#id]
```

Lightweight version query; does not probe hardware.

Example:

```
VER
OK ver fw=0.20260602.6 proto=2
```

### HELP

```
HELP [#id]
→ OK help PING ECHO ID VER HELP SET GET STREAM SNAP S T D G STOP GRIP ZERO OI OZ OR OP OV OL OA P PA [#id]
```

Returns a space-separated list of all implemented verbs in a single `OK
help` response.

---

## 7. Config: `SET` / `GET`

### GET

```
GET [<key>…] [#id]
→ CFG <key>=<value>… [#id]
```

With no arguments, dumps all registered keys.  With one or more key
names, returns only those keys.  For each unknown key a separate `ERR
badkey <key>` is emitted (does not prevent the CFG line from being sent
for valid keys).

Examples:

```
GET
CFG ml=0.487 mr=0.481 kff=0.150 klf=1.000 klb=1.000 krf=1.000 krb=1.000 adjThr=0.500 adjGain=0.050 tw=120 pid.kp=300.000 pid.ki=0.000 pid.kd=0.000 pid.max=30.000 turnThr=50 doneTol=5 distScale=0.940 turnScale=1.070 minSpeed=50 sTimeout=200 tick=20 tlmPeriod=0

GET ml pid.kp
CFG ml=0.487 pid.kp=300.000

GET ml #9
CFG ml=0.487 #9

GET badkey
ERR badkey badkey
```

### SET

```
SET <key>=<value>… [#id]
→ OK set <applied-key>=<value>… [#id]
   [ERR badkey <key> [#id]]…
```

Applies each valid key immediately to the live config.  Unknown keys
each produce a separate `ERR badkey` line; valid keys are applied and
listed in the `OK set` response body.  If no valid keys are provided,
only `ERR` lines are emitted (no `OK set`).

Changing any of `pid.kp`, `pid.ki`, `pid.kd`, or `pid.max` calls
`MotorController::updatePidGains()` immediately.

Examples:

```
SET ml=0.487 mr=0.481
OK set ml=0.487 mr=0.481

SET ml=0.487 bad=1
OK set ml=0.487
ERR badkey bad

SET bad=1
ERR badkey bad

SET
ERR badarg no key=value pairs
```

### Named Key Table

All 22 registered config keys, their types, defaults, and the v1
equivalents they replace.

| Key         | Type        | Wire format | Default  | Meaning                                 | v1 equiv  |
|-------------|-------------|-------------|----------|-----------------------------------------|-----------|
| `ml`        | float       | `%.3f`      | `0.487`  | mm per degree of rotation, left wheel   | `KCL`     |
| `mr`        | float       | `%.3f`      | `0.481`  | mm per degree of rotation, right wheel  | `KCR`     |
| `kff`       | float       | `%.3f`      | `0.150`  | Feed-forward gain                       | `KFF`     |
| `klf`       | float       | `%.3f`      | `1.000`  | Left-forward motor scale factor         | `KLF`     |
| `klb`       | float       | `%.3f`      | `1.000`  | Left-backward motor scale factor        | `KLB`     |
| `krf`       | float       | `%.3f`      | `1.000`  | Right-forward motor scale factor        | `KRF`     |
| `krb`       | float       | `%.3f`      | `1.000`  | Right-backward motor scale factor       | `KRB`     |
| `adjThr`    | float       | `%.3f`      | `0.500`  | Slower-wheel adjustment threshold       | —         |
| `adjGain`   | float       | `%.3f`      | `0.050`  | Slower-wheel adjustment gain            | —         |
| `tw`        | float-as-int| `%d`        | `120`    | Track width in mm                       | `KAT`     |
| `pid.kp`    | float       | `%.3f`      | `300.000`| Ratio PID proportional gain             | `KCP`     |
| `pid.ki`    | float       | `%.3f`      | `0.000`  | Ratio PID integral gain                 | `KCI`     |
| `pid.kd`    | float       | `%.3f`      | `0.000`  | Ratio PID derivative gain               | `KCD`     |
| `pid.max`   | float       | `%.3f`      | `30.000` | Ratio PID output clamp                  | `KCM`     |
| `turnThr`   | float-as-int| `%d`        | `50`     | Go-to pre-rotate threshold (mm/deg)     | `KGT`     |
| `doneTol`   | float-as-int| `%d`        | `5`      | Go-to done tolerance (mm)               | `KGD`     |
| `distScale` | float       | `%.3f`      | `0.940`  | Distance command scale factor           | `KDS`     |
| `turnScale` | float       | `%.3f`      | `1.070`  | Turn command scale factor               | `KTS`     |
| `minSpeed`  | int32       | `%d`        | `50`     | Minimum drive speed (mm/s)              | `KMS`     |
| `sTimeout`  | int32       | `%d`        | `200`    | Streaming watchdog timeout (ms)         | `KST`     |
| `tick`      | int32       | `%d`        | `20`     | Main-loop tick period (ms)              | `KTK`     |
| `tlmPeriod` | int32       | `%d`        | `0`      | TLM streaming period (ms); 0 = off      | —         |

Type `float-as-int`: stored internally as `float`, read/written on the
wire as a decimal integer (no fractional part).  `SET tw=121` writes
`121.0f`; `GET tw` returns `121`.

Value conventions:
- All distances are integer millimetres; no implicit scaling, no `×10`
  multipliers.
- Float keys use three decimal places on output (`%.3f`).
- Integer and float-as-int keys use `%d` on output.
- `SET` accepts float text for float keys (`atof`) and integer text for
  int and float-as-int keys (`atoi`).

---

## 8. Telemetry: `TLM` Frame

### STREAM

```
STREAM <ms> [#id]
→ OK stream period=<ms> [#id]

STREAM fields=<field>,… [#id]
→ OK stream fields=<field>,… [#id]
```

`STREAM <ms>` sets the periodic telemetry interval in milliseconds.
`ms=0` disables streaming.  The minimum enforced period is 20 ms;
smaller positive values are clamped to 20.

`STREAM fields=<csv>` sets the field subscription bitmask.  The value
is a comma-separated list of field names (`enc`, `pose`, `vel`, `line`,
`color`).  Any unrecognised name is silently ignored.  An empty or
all-unrecognised list resets the mask to `TLM_FIELD_ALL` (all fields).

Examples:

```
STREAM 100
OK stream period=100

STREAM 0
OK stream period=0

STREAM fields=enc,pose
OK stream fields=enc,pose

STREAM fields=enc,pose,line
OK stream fields=enc,pose,line
```

### SNAP

```
SNAP [#id]
→ OK snap [#id]
```

Sets a one-shot flag; the next `Robot::tick()` call emits one immediate
TLM frame before clearing the flag.  The `OK snap` response is returned
immediately (before the TLM frame arrives).

### TLM Frame Format

```
TLM t=<ms> mode=<char> [enc=<l>,<r>] [pose=<x>,<y>,<h>] [vel=<vl>,<vr>] [line=<g1>,<g2>,<g3>,<g4>] [color=<r>,<g>,<b>,<c>]
```

Fields are emitted in the order shown; fields whose subscription bit is
clear, or whose hardware is absent, are omitted.

| Field    | Format                      | Units / notes                                            |
|----------|-----------------------------|----------------------------------------------------------|
| `t`      | `%lu` (unsigned long)       | Robot clock in ms at sensor-sample time (see note below) |
| `mode`   | single character            | `I`=idle, `S`=streaming, `T`=timed, `D`=distance, `G`=go-to |
| `enc`    | `%d,%d`                     | Left and right encoder accumulated distance in mm        |
| `pose`   | `%d,%d,%d`                  | x mm, y mm, heading in centi-degrees                     |
| `vel`    | `%d,%d`                     | Left and right actual velocity in mm/s (deferred Sprint 010) |
| `line`   | `%u,%u,%u,%u`               | Four greyscale channels (raw ADC counts)                 |
| `color`  | `%u,%u,%u,%u`               | R, G, B, clear channels (raw ADC counts)                 |

**Timestamp discipline.** `t=` is captured at the start of sensor
reading (before `snprintf`), not at line-send time.  This ensures the
translated host time reflects when the measurements were taken, not the
variable send latency.

**Pose source.** When the OTOS sensor is present and detected at boot,
`pose=` values come from `OtosSensor::getPositionRaw()`.  Otherwise
they come from the dead-reckoning odometry integrator.

**`vel=` field.** The field bitmask bit is defined (`TLM_FIELD_VEL =
0x04`) and the STREAM fields parser recognises `vel`, but the
`MotorController::getActualVelocity()` data path is deferred to Sprint
010.  The field will be absent from TLM frames until that sprint lands.

Example:

```
TLM t=12345 mode=S enc=1024,1019 pose=350,-12,1780 line=120,340,330,118 color=21,30,18,80
TLM t=12400 mode=I enc=1024,1019 pose=350,-12,1780
```

---

## 9. Time Synchronisation

The robot runs a free-running clock (`uBit.systemTime()`, milliseconds
since boot).  The robot clock is never set from the host; setting it
would corrupt odometry `dt` computations and is unnecessary.

Instead, the host estimates the *offset* (and optionally the *skew*)
between host-monotonic time and robot time, then translates robot `t=`
timestamps into host time for event correlation.

### Algorithm (NTP-style min-RTT filtering)

For each PING exchange the host records:
- `T0` — host monotonic time (ms) immediately *before* sending `PING`
- `T1` — host monotonic time (ms) immediately *after* receiving the reply
- `t_r` — robot clock stamp (ms) from `OK pong t=<t_r>`

Assuming a roughly symmetric link delay:

```
offset_ms = (T0 + T1) / 2 − t_r
```

Fire N PINGs (default 5) and keep the sample with the smallest RTT
(= T1 − T0).  The minimum-RTT sample has the least relay/queuing
jitter.  Accuracy is bounded by approximately half the minimum RTT.

### Skew Compensation

After accumulating samples spanning at least 1 ms of robot time, the
host fits a linear model by ordinary least squares:

```
host_mid ≈ a · t_robot + b
```

`a` is the skew factor (ideally ≈ 1.0); `b` is the intercept.  The
`to_host_time()` function uses this model when available, falling back
to the offset-only estimate.  Re-sync (new `ping_burst()`) is
recommended every 30–60 s to track micro:bit crystal drift (~tens of
ppm, a few ms/min).

### Host-side API (`ClockSync`)

```python
cs = ClockSync()
cs.ping_burst(lambda cmd: proto.ping_and_raw(cmd))  # fire 5 PINGs
host_ms = cs.to_host_time(tlm_frame.t)              # translate robot timestamp
```

`ClockSync.stale(max_age_s=60.0)` returns `True` if no burst has been
recorded within `max_age_s` seconds.

---

## 10. Motion Commands

Motion commands are asynchronous: the firmware returns `OK …` immediately
(acknowledging the command parameters) and later sends an `EVT done …`
or `EVT safety_stop` when the drive ends.

When the originating T, D, or G command carried a `#id`, that id is echoed
on the asynchronous completion event.  Commands issued without a `#id`
produce bare events with no `#id`.

### EVT Completion Events

| Event                  | Emitted when                                          |
|------------------------|-------------------------------------------------------|
| `EVT done T [#id]`     | Timed drive elapsed                                   |
| `EVT done D [#id]`     | Distance drive target reached (or 5-second timeout)   |
| `EVT done G [#id]`     | Go-to arc completed within `doneTol` mm               |
| `EVT safety_stop [#id]`| S-mode watchdog expired (no S command within `sTimeout` ms) |

`[#id]` is present only when the originating command carried one.  Example:

```
T 200 200 1000 #12
OK drive l=200 r=200 ms=1000 #12
… (later) …
EVT done T #12
```

Bare form (no corr id):

```
T 200 200 1000
OK drive l=200 r=200 ms=1000
EVT done T
```

### S — Streaming (Watchdog) Drive

```
S <l> <r> [#id]
→ OK drive l=<l> r=<r> [#id]
```

Sets left and right wheel velocities (mm/s) and resets the streaming
watchdog.  If no `S` command arrives within `sTimeout` ms (default 200),
the firmware stops the motors and emits `EVT safety_stop`.

Velocity range: −1000 … +1000 mm/s per wheel.  Values outside this
range return `ERR range l` or `ERR range r`.

Example:

```
S 200 150
OK drive l=200 r=150

S -100 100
OK drive l=-100 r=100
```

### T — Timed Drive

```
T <l> <r> <ms> [#id]
→ OK drive l=<l> r=<r> ms=<ms> [#id]
  … (later, asynchronously) …
  EVT done T [#id]
```

Drives at the given speeds for `ms` milliseconds (1 … 30 000).

Velocity range: −1000 … +1000 mm/s.  Duration range: 1 … 30 000 ms.

Example:

```
T 200 200 1000
OK drive l=200 r=200 ms=1000
EVT done T

T 200 200 1000 #12
OK drive l=200 r=200 ms=1000 #12
EVT done T #12
```

### D — Distance Drive

```
D <l> <r> <mm> [#id]
→ OK drive l=<l> r=<r> mm=<mm> [#id]
  … (later, asynchronously) …
  EVT done D [#id]
```

Drives at the given speeds until the average of the absolute encoder
travel on both wheels reaches `mm` millimetres (1 … 10 000), or until
a 5-second hard timeout fires.

Velocity range: −1000 … +1000 mm/s.  Distance range: 1 … 10 000 mm.

Example:

```
D 200 200 300
OK drive l=200 r=200 mm=300
EVT done D

D 200 200 300 #5
OK drive l=200 r=200 mm=300 #5
EVT done D #5
```

### G — Go-To (relative XY)

```
G <x> <y> <speed> [#id]
→ OK goto x=<x> y=<y> speed=<speed> [#id]
  … (later, asynchronously) …
  EVT done G [#id]
```

Navigate to the relative XY point `(x, y)` (mm) at the given
`speed` (mm/s).  The coordinate system is robot-relative: +x is forward,
+y is left.  Heading is in centi-degrees.

The firmware optionally pre-rotates when the bearing angle exceeds
`turnThr` mm (default 50), then drives an arc to the target.  The
`G` verb is unambiguously go-to; the gripper is controlled by `GRIP`.

Coordinate range: −10 000 … +10 000 mm per axis.
Speed range: 1 … 1 000 mm/s.

Example:

```
G 300 0 200
OK goto x=300 y=0 speed=200
EVT done G

G 300 0 200 #7
OK goto x=300 y=0 speed=200 #7
EVT done G #7
```

### STOP

```
STOP [#id]
→ OK stop [#id]
```

Stops motors immediately.  Clears any active drive mode.  No `EVT` is
emitted.

Example:

```
STOP
OK stop
```

### GRIP — Gripper Control

```
GRIP <deg> [#id]   → OK grip deg=<deg> [#id]   (set angle)
GRIP       [#id]   → OK grip deg=<deg> [#id]   (query current angle)
```

Sets the gripper servo to the given angle (0 … 180 degrees).  With no
argument, returns the last commanded angle (0 at power-on).

Range: 0 … 180 degrees.

Example:

```
GRIP 90
OK grip deg=90

GRIP
OK grip deg=90
```

### ZERO — Zero Encoders / Odometry

```
ZERO enc          [#id]  → OK zero enc [#id]
ZERO pose         [#id]  → OK zero pose [#id]
ZERO enc pose     [#id]  → OK zero enc pose [#id]
```

`enc` resets the encoder accumulators (calls
`MotorController::resetEncoderAccumulators()`).  `pose` resets the
odometry integrator to `(0, 0, 0)` (calls `Odometry::zero()`).  Both
may be specified in one command.

At least one of `enc` or `pose` must be present; otherwise `ERR badarg`.

Example:

```
ZERO enc
OK zero enc

ZERO enc pose
OK zero enc pose
```

---

## 11. OTOS / Port I/O Commands

### OI — OTOS Init

```
OI [#id]
→ OK oi [#id]
   ERR nodev oi [#id]   (if OTOS not detected at boot)
```

Calls `OtosSensor::init()` to re-initialise OTOS signal processing.

### OZ — OTOS Zero Position

```
OZ [#id]
→ OK oz [#id]
   ERR nodev oz [#id]
```

Zeroes the OTOS world-frame position to the current location
(`setPositionRaw(0, 0, 0)`).

### OR — OTOS Reset Tracking

```
OR [#id]
→ OK or [#id]
   ERR nodev or [#id]
```

Resets OTOS Kalman filter state (`resetTracking()`).

### OP — OTOS Read Position

```
OP [#id]
→ OK pos x=<x> y=<y> h=<h> [#id]
   ERR nodev op [#id]
```

Returns the current OTOS world-frame position.  `x` and `y` are in
mm; `h` is the heading in centi-degrees (integer).

Example:

```
OP
OK pos x=350 y=-12 h=1780
```

### OV — OTOS Set Position

```
OV <x> <y> <h> [#id]
→ OK setpos x=<x> y=<y> h=<h> [#id]
   ERR nodev ov [#id]
```

Sets the OTOS world-frame position (`setPositionRaw(x, y, h)`).

Example:

```
OV 0 0 0
OK setpos x=0 y=0 h=0
```

### OL — OTOS Linear Scalar

```
OL <val> [#id]   → OK linear scalar=<val> [#id]   (set)
OL       [#id]   → OK linear scalar=<val> [#id]   (read)
   ERR nodev ol [#id]
```

Gets or sets the OTOS linear scalar calibration register (`int8_t`).

### OA — OTOS Angular Scalar

```
OA <val> [#id]   → OK angular scalar=<val> [#id]   (set)
OA       [#id]   → OK angular scalar=<val> [#id]   (read)
   ERR nodev oa [#id]
```

Gets or sets the OTOS angular scalar calibration register (`int8_t`).

### P — Digital Port Read / Write

```
P <port>        [#id]  → OK port p=<port> v=<val> [#id]   (read)
P <port> <val>  [#id]  → OK port p=<port> v=<val> [#id]   (write)
```

`port` is 1–4 (J-port number).  On read, `v` is 0 or 1.  On write,
any non-zero `val` is treated as 1.

Examples:

```
P 1
OK port p=1 v=0

P 2 1
OK port p=2 v=1
```

### PA — Analog Port Read / Write

```
PA <port>        [#id]  → OK aport p=<port> v=<val> [#id]   (read)
PA <port> <val>  [#id]  → OK aport p=<port> v=<val> [#id]   (write)
```

`port` is 1–4.  `val` is in the range 0–1023 (10-bit ADC or PWM).
A write `val` outside 0–1023 returns `ERR range val`.

Examples:

```
PA 3
OK aport p=3 v=512

PA 3 256
OK aport p=3 v=256
```

---

## 12. Buffer and Framing Note

The firmware uses a 512-byte line buffer (`buf[512]` in `main.cpp`,
`REASM_MAX = 512` in `Radio.h`).  The maximum message size is therefore
511 bytes (one byte for the NUL terminator).

A full `GET` dump of all 22 keys is approximately 290 bytes — well
within this limit.  Large `SET` commands or long `ECHO` payloads are
similarly accommodated.

The 20 ms minimum telemetry period (`STREAM`) prevents buffer overrun
from rapid TLM frames.

The RAW250 radio transport uses a 247-byte MTU per fragment; the HAL
transparently fragments and reassembles messages of up to `REASM_MAX`
bytes.  The protocol layer is not aware of fragmentation.

---

## 13. Verification Examples

### ECHO Round-Trip

```
ECHO the-quick-brown-fox-jumped-over-the-lazy-dog
OK echo the-quick-brown-fox-jumped-over-the-lazy-dog
```

A ~200-byte payload tests reassembly in both directions over the relay.

### GET Dump (all keys)

```
GET
CFG ml=0.487 mr=0.481 kff=0.150 klf=1.000 klb=1.000 krf=1.000 krb=1.000 adjThr=0.500 adjGain=0.050 tw=120 pid.kp=300.000 pid.ki=0.000 pid.kd=0.000 pid.max=30.000 turnThr=50 doneTol=5 distScale=0.940 turnScale=1.070 minSpeed=50 sTimeout=200 tick=20 tlmPeriod=0
```

### SET and Verify

```
SET ml=0.490 mr=0.485
OK set ml=0.490 mr=0.485

GET ml mr
CFG ml=0.490 mr=0.485
```

### TLM Frame

```
SNAP
OK snap
TLM t=12345 mode=I enc=0,0 pose=0,0,0 line=120,340,330,118 color=21,30,18,80
```

### Motion End-to-End

Without correlation id:

```
D 200 200 300
OK drive l=200 r=200 mm=300
TLM t=12400 mode=D enc=45,44 pose=45,0,0
TLM t=12420 mode=D enc=89,88 pose=89,0,0
EVT done D
```

With correlation id (host can match completion to originating request):

```
D 200 200 300 #5
OK drive l=200 r=200 mm=300 #5
TLM t=12400 mode=D enc=45,44 pose=45,0,0
EVT done D #5
```

### Clock-Sync Alignment

After a `ping_burst()` of 5 samples:

```python
cs = ClockSync()
cs.ping_burst(lambda cmd: proto._conn.send(cmd, read_ms=500)
              .get("responses", [""])[0])

# Translate a TLM timestamp to host time:
host_ms = cs.to_host_time(tlm_frame.t)

# Check staleness:
if cs.stale(max_age_s=60.0):
    cs.ping_burst(...)
```

Accuracy: within approximately half the minimum RTT (typically a few ms
over the half-duplex relay).

---

## Appendix: Removed v1 Commands

The following v1 command vocabulary is removed in v2.  Any of these
verbs returns `ERR unknown <verb>`.

| Removed verb / prefix    | v1 meaning                             |
|--------------------------|----------------------------------------|
| `K*` (e.g. `KCP`, `KCL`) | Per-constant calibration set/get       |
| `ENC`                    | Encoder query                          |
| `SO`                     | Sensor output (legacy)                 |
| `SSE`, `SSO`, `SSC`, `SSL` | Streaming sensor commands             |
| `HELLO`                  | Legacy identity / handshake            |
| `DEVICE:`                | Legacy device prefix format            |
| `X`                      | Legacy stop/reset                      |
| Packed motion (`S+200-150`) | Sign-prefix packed speed arguments  |
