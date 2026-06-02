# On-Chip Debugging with pyOCD

How to debug the firmware running on real hardware — set breakpoints, step
source, inspect memory and peripheral registers, and recover a bricked chip.
This is **live debugging on the robot**, complementary to the unit tests and the
[hardware bench gate](hardware-bench-testing.md).

> **For agents:** read [Agent guidance](#agent-guidance) before you start. The
> short version: the board is on a stand and safe to drive; everything runs
> through `just` recipes; `gdb` sessions must be driven **non-interactively**
> (batch mode), never as a blocking REPL.

## The hardware path

The robot is a **micro:bit V2** = a Nordic **nRF52833** (Cortex-M4F) application
chip plus an on-board **interface chip (KL27) running DAPLink**. When you plug in
USB, DAPLink exposes three things:

- a **CMSIS-DAP** debug probe (SWD access to the nRF52833),
- a USB mass-storage drive (drag-drop `.hex` flashing), and
- a USB serial port (the robot's command/telemetry link).

**pyOCD** talks to the CMSIS-DAP probe over **SWD** natively — no external
debug adapter, no OpenOCD required. It can flash, reset, read/write memory and
registers, and host a **GDB remote server** that `arm-none-eabi-gdb` (and the
VS Code cortex-debug extension) connect to for source-level debugging.

```
arm-none-eabi-gdb ──┐
                    ├─(GDB remote :3333)─→ pyOCD ─(CMSIS-DAP/SWD)─→ DAPLink ─→ nRF52833
VS Code cortex-debug┘
```

The same pyOCD is what `mbdeploy` already uses to flash the board (see
`mbdeploy/src/mbdeploy/cli.py`), so if `mbdeploy deploy` works, debugging works.

## Prerequisites

- **pyOCD on PATH** — `pyocd --version` should report `0.44.1` or later. It is
  also bundled in `mbdeploy/.venv` (a declared dependency); if it is not on your
  PATH, prefix commands with `mbdeploy/.venv/bin/python -m pyocd …`.
- **arm-none-eabi-gdb** — installed via `just setup-macos` / `just link-arm-tools`
  (lives at `/opt/homebrew/bin/arm-none-eabi-gdb`).
- **A connected micro:bit V2.** Confirm with `pyocd list` — you should see
  `Arm BBC micro:bit CMSIS-DAP … ✔︎ nrf52833`. No board = nothing to debug.
- **A current build** at `build/MICROBIT` (the ELF, with debug symbols). Build
  with `just build`. The ELF is `not stripped` and carries `debug_info`.

## Quick start

The whole workflow is wrapped in `just` recipes:

| Recipe          | What it does                                                        |
|-----------------|---------------------------------------------------------------------|
| `just debug`    | Start the pyOCD GDB server on `:3333` (`--persist`). Leave running. |
| `just gdb`      | Attach gdb to that server, flash, reset, stop at `main()`.          |
| `just commander`| Interactive pyOCD console — read/write registers, memory, peripherals. |
| `just erase`    | CTRL-AP mass erase to recover an APPROTECT-locked nRF52.            |

### CLI flow (two terminals)

```bash
# terminal 1 — start the debug server and leave it running
just debug

# terminal 2 — attach, load the current build, reset, break at main()
just gdb
```

`just debug` runs `pyocd gdbserver -t nrf52833 --persist`. `--persist` keeps the
server alive across client disconnects and reflashes, so you can detach and
reattach gdb without restarting it. Stop it with Ctrl-C when done.

`just gdb` runs `arm-none-eabi-gdb build/MICROBIT` with these commands:
`target remote :3333`, `load` (flash the ELF), `monitor reset halt`,
`break main`, `continue`. You land at `main()` with full source.

### VS Code flow

[.vscode/launch.json](../.vscode/launch.json) defines cortex-debug configs (the
`marus25.cortex-debug` extension is recommended in
[.vscode/extensions.json](../.vscode/extensions.json)):

- **micro:bit PyOCD Cortex Debug** — launch: cortex-debug spawns its own pyOCD
  server, flashes, and stops. Use this for a normal F5 debug session.
- **(attach) micro:bit PyOCD Cortex Debug** — attach to firmware already
  running (e.g. after `just debug`) without reflashing.

Both use `device: nrf52833` and the `nrf52833.svd` so the **peripheral register
view** decodes named registers (GPIO, TWI/I2C, TIMER, RADIO, …).

## Inspecting without a full GDB session

`just commander` (`pyocd commander -t nrf52833`) is faster than gdb for
poking at state. Inside the console:

```
reset halt          # stop the core
reg                 # dump core registers
read32 0x20000000   # read a RAM word
read8 0x40002000    # peripheral register byte
write32 0x... 0x... # write memory
peripheral RADIO    # decode a named peripheral's registers (uses the SVD)
```

## Agent guidance

This section is the contract for autonomous agents debugging the board.

**1. Preconditions, every time.** Before any debug action:
   - `pyocd list` → confirm exactly one micro:bit V2 is connected. If zero,
     stop and report; you cannot debug without hardware.
   - The robot is **mounted on a stand with wheels off the ground** (see
     [hardware-bench-testing.md](hardware-bench-testing.md)), so it is safe to
     run motors while halted/stepping. Do not assume it is safe to drive if that
     doc ever says otherwise.

**2. The server is long-running; the client is not.** `just debug` blocks
   forever. Start it as a **background process** and capture its log, e.g.

   ```bash
   just debug > /tmp/pyocd-gdb.log 2>&1 &   # background; note the PID
   sleep 6                                   # wait for "GDB server listening on port 3333"
   ```

   Always **kill the server** when finished (`kill <pid>`); a lingering server
   holds the SWD port and blocks the next session and `mbdeploy deploy`.

**3. Drive gdb non-interactively — never block on a REPL.** Use batch mode with
   explicit commands and `--batch` so the process exits on its own. Example:
   inspect a global and the call stack, then quit, without human input:

   ```bash
   arm-none-eabi-gdb -q --batch build/MICROBIT \
     -ex "target remote :3333" \
     -ex "monitor reset halt" \
     -ex "break main" -ex "continue" \
     -ex "info registers" \
     -ex "backtrace" \
     -ex "detach"
   ```

   For temporary probing, prefer reading via `pyocd commander` one-liners or
   `gdb --batch` over an interactive session you cannot type into.

**4. One consumer of the SWD port at a time.** pyOCD, a running `gdbserver`, and
   `mbdeploy deploy` all need the probe. Don't flash while a gdbserver is up;
   stop the server first. Port `:3333` already in use means a stale server —
   find and kill it (`lsof -nP -iTCP:3333 -sTCP:LISTEN`).

**5. Report faithfully.** If the board isn't connected, if attach fails, or if
   you mass-erased the part, say so explicitly with the tool output. Don't claim
   a breakpoint was hit if the session never reached it.

## Gotchas

- **APPROTECT lock.** nRF52 parts can boot with debug access disabled
  (`… not in secure state` is normal/fine; a hard *locked* part rejects all SWD
  reads and every flash-erase). Recover with `just erase` (CTRL-AP mass erase),
  then reflash. `mbdeploy` does this automatically on a failed flash; for raw
  debugging you may need to do it by hand.
- **Optimization makes stepping jumpy.** CODAL builds at `-Os`, so single-step
  jumps around and some locals show `<optimized out>`. For serious source
  debugging, build a `-Og -g3` variant. (Not wired into the build yet — ask
  before changing global compiler flags.)
- **Stale DAPLink firmware** can be flaky with pyOCD. If attach is unreliable,
  update the interface firmware: drag the latest DAPLink `.hex` onto the
  `MAINTENANCE` drive.
- **The serial link and the debugger coexist.** SWD debugging does not interfere
  with the USB serial command/telemetry port — you can keep a host serial
  session open while debugging, though a halted core won't service commands.

## Reference

- Recipes: [justfile](../justfile) (`debug`, `gdb`, `commander`, `erase`)
- VS Code configs: [.vscode/launch.json](../.vscode/launch.json)
- SVD (peripheral decode): `libraries/codal-nrf52/nrfx/mdk/nrf52833.svd`
- Flash/deploy tooling that shares pyOCD: `mbdeploy/src/mbdeploy/cli.py`
- pyOCD docs: <https://pyocd.io>
