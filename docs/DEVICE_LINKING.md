# Device Linking: picking the right micro:bit to flash

This explains how the deploy tooling decides **which physical micro:bit is
which** when several are plugged in at once (e.g. a robot *and* a radio
relay), so it can flash each one with firmware for its type.

It is written to be transferable: to use this in another repo, copy
[`scripts/lib/device-link.js`](../scripts/lib/device-link.js) and, optionally,
[`scripts/device-map.js`](../scripts/device-map.js) (the diagnostic tool).

---

## The core problem

You learn a device's **type** from one place and **flash** it from another:

| You want | Where it comes from | USB interface |
|---|---|---|
| The **type** (`ROBOT`, `RADIORELAY`, …) | the firmware's `DEVICE:` announcement, read over the **serial port** | CDC |
| To **flash** firmware | a `.hex` written via **CMSIS-DAP** (dapjs) or dropped on the **MSD volume** | HID / MSD |

With one board this is trivial. With two boards you must *prove* that "the
port I read `RADIORELAY` from" and "the DAP/volume I'm about to flash" are the
**same physical device** — otherwise you can flash robot firmware onto the relay.

## The key insight

A micro:bit presents to the host as **one composite USB device with several
interfaces**, and every interface carries the **same USB serial number**:

```
                       ┌───────────────────────── one micro:bit ─────────────────────────┐
                       │  USB serial number: 9906360200052820e9d16c3809a44554000000006e052820 │
                       │  (== "Unique ID" in DETAILS.TXT, == DAPLink web page id)         │
                       ├──────────────┬───────────────────────────┬────────────────────────┤
   interface:          │  CDC serial  │  HID / CMSIS-DAP          │  MSD mass storage      │
   host sees:          │ /dev/cu.usb… │  HID.devices()[].path     │  /Volumes/MICROBIT     │
   gives you:          │  the TYPE    │  flash via dapjs          │  flash via .hex copy   │
                       └──────────────┴───────────────────────────┴────────────────────────┘
```

**That shared USB serial number is the join key.** Match the port, the DAP
handle, and the volume by serial and you have one coherent device.

### Why the firmware can't just print the serial

The USB serial belongs to the **interface chip** (the KL27 that runs DAPLink
and talks USB). The user's TypeScript runs on the **nRF52833 target**, a
*different* chip. `control.deviceSerialNumber()` and `control.deviceName()`
read the nRF52's id (e.g. `f2fc76f3` / `zavaz`) — these have **no numeric
relationship** to the USB serial. The target has no API to read the interface
chip's serial, so the firmware **cannot** emit it. The host must supply the link.

> This is why the announcement's last field (`f2fc76f3`) is *not* used for
> matching. It's extra identity, not a join key.

## How the host gets the serial for each face

| Face | How we read its serial |
|---|---|
| HID / CMSIS-DAP | `node-hid` reports `serialNumber` directly. This is the authoritative list of connected micro:bits. |
| MSD volume | Read `/Volumes/MICROBIT*/DETAILS.TXT` → `Unique ID:` line. |
| **CDC serial port** | **macOS:** `ioreg -r -c IOUSBHostDevice -l` lists each USB device's `"USB Serial Number"` together with the `IOCalloutDevice` (`/dev/cu.*`) and `IODialinDevice` (`/dev/tty.*`) of its nested serial interface. |

The serial port is the tricky one — the OS port name (`usbmodem21421202`) is
derived from USB *topology*, not the serial, so you must ask IOKit. That's
what `portSerialMap()` does.

> **Do not match the serial port by enumeration order.** HID enumeration order
> and `usbmodem` numbering are independent; pairing them by array index works
> with one board and silently mis-pairs with two. This was the original bug in
> `devices.js`.

### Other platforms

`portSerialMap()` returns `{}` off macOS, so ports degrade to "unlinked"
rather than mis-linked. To port it:

- **Linux:** the serial is in the `/dev/serial/by-id/` symlink name, or via
  `udevadm info -q property -n /dev/ttyACMx` → `ID_SERIAL_SHORT`.
- **Windows:** the serial appears in the device instance path via WMI /
  `SetupAPI`.

HID and MSD matching are already cross-platform.

## The join, in code

[`scripts/lib/device-link.js`](../scripts/lib/device-link.js) exposes:

```js
const link = require("./lib/device-link");

// Every connected micro:bit, all sources joined on the USB serial:
const devices = await link.enumerateDevices({ announce: true });
// → [{ serial:{full,short,display}, hidPath, cuPort, ttyPort,
//      volume, flashError, announcement:{type,name,deviceName,nrfSerial} }]
```

`enumerateDevices`:

1. `findHidDevices()` — authoritative serial list (what we can flash).
2. `portsBySerial(serials)` — ioreg port→serial, filtered to known micro:bit
   serials so non-micro:bit ports can never be mis-attributed.
3. `volumesBySerial()` — DETAILS.TXT Unique ID → volume.
4. With `announce:true`, opens each port, sends `HELLO`, and parses the
   `DEVICE:` line for the **type**.

## The deploy flow

```
1. devices = enumerateDevices({ announce: true })
2. target  = devices.find(d => d.announcement?.type === "RADIORELAY")
3. flash the firmware to target  →  dapjs(target.hidPath)
                                     or copy .hex to target.volume
```

Because steps 2 and 3 reference the **same `target` object**, the type you
matched on and the device you flash are guaranteed to be the same physical
board. No index guessing, no ambiguity with multiple devices.

> **Type names in this repo.** The snippet above flashes the relay, matching
> on its type. In *this* project the deploy script does the inverse — it flashes
> the **robot** and must *avoid* the relay. The robot announces
> `DEVICE:Nezha2:…` and the relay announces `DEVICE:RADIOBRIDGE:relay:zavaz:…`
> (the generic `RADIORELAY` above is illustrative — the real role string is
> `RADIOBRIDGE`). The Python guard matches the relay on either token
> (`RELAY`/`BRIDGE`) so both spellings are caught.

### Sending HELLO

Reading the type requires the firmware to re-announce on request. The
[`announce`](../src/announce.ts) namespace supports this: call
`announce.listenForHello()` (or route incoming lines through
`announce.handleHello(line)`) so that a host writing `HELLO\n` triggers a fresh
`DEVICE:` line. The relay does this at [`src/main.ts:109`](../src/main.ts#L109).
Without it, the announcement is only emitted once at boot and may have scrolled
past by the time the host connects.

> The `TYPE` column is **best-effort**: it depends on the firmware answering
> HELLO and the port being free at that instant (a serial monitor or the
> bridge holding the port will block the read). The serial-based **join** of
> port/volume/DAP is always reliable; only the type read can come back blank,
> in which case re-run, or fall back to the bridge's recorded type.

## Tools

```bash
npm run device-map          # step-by-step: each raw source + the joined table
npm run devices -- --announce   # the normal listing, with the type badge
npm run devices -- --json --announce   # machine-readable, for deploy scripts
```

`npm run device-map` is the one to look at first — it prints each enumeration
source (HID, ioreg ports, MSD volumes) separately and then the joined table,
so you can see exactly which serial ties what together. Example with a relay
and a speed-test rig both plugged in:

```
4. Joined devices  (all sources matched on the USB serial)
   id (last 6)  TYPE         cu port                    volume       DAP
   a44554       RADIORELAY   /dev/cu.usbmodem21421202   MICROBIT 1   yes
   0cfd6c       SPEEDTEST    /dev/cu.usbmodem2142202    MICROBIT     yes
```

## The deploy guard (this repo)

This project's deploy tooling is Python, and the serial-based join is
implemented in [`scripts/lib/device_link.py`](../scripts/lib/device_link.py) —
the Python mirror of `device-link.js`. [`scripts/deploy.py`](../scripts/deploy.py)
uses it as a **safety gate before any USB flash**:

- It resolves the target volume → USB serial (DETAILS.TXT) → port (`ioreg`) →
  type (HELLO probe), exactly the join above.
- If the resolved device is the **radio relay** (`is_relay()` matches
  `RADIOBRIDGE`/`RADIORELAY`), it **refuses to flash — with no `--force`
  override**. The relay is never overwritten.
- With no `--usb-mount`, it **auto-selects the robot** (`Nezha2`) and skips a
  connected relay. Multiple robots, or an unconfirmable type, stop the deploy
  rather than guess (pass `--usb-mount`, and `--force` only to accept an
  *unknown* — never a relay).

```bash
uv run python3 scripts/deploy.py                 # auto-detect the robot, refuse the relay
uv run python3 scripts/deploy.py --usb-mount /Volumes/MICROBIT
```

## Relation to the Python discovery code

This repo also has a Python relay stack ([`radio_relay/discovery.py`](../radio_relay/discovery.py)).
Its `find_relays()` does the HELLO probe to learn each port's **type**, but it
stops there — `RelayInfo` carries `port` and the nRF `serial` (parsed from the
announcement), **not** the USB serial. So it can tell you the type on a port but
cannot, on its own, join that to the volume/DAP handle you flash.
`scripts/lib/device_link.py` is the module that *does* complete that join (via
`portSerialMap()`'s Python twin, `port_serial_map()`); the Node module
[`scripts/lib/device-link.js`](../scripts/lib/device-link.js) remains the
original reference implementation.
