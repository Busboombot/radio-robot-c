# microbit-v2-samples

[![Native Build Status](https://github.com/lancaster-university/microbit-v2-samples/actions/workflows/build.yml/badge.svg)](https://github.com/lancaster-university/microbit-v2-samples/actions/workflows/build.yml) [![Docker Build Status](https://github.com/lancaster-university/microbit-v2-samples/actions/workflows/docker-image.yml/badge.svg)](https://github.com/lancaster-university/microbit-v2-samples/actions/workflows/docker-image.yml)

This repository provides the necessary tooling to compile a C/C++ CODAL program for the micro:bit V2 and generate a HEX file that can be downloaded to the device.

## Python host library — robot_radio

`host/robot_radio/` is the **canonical, tested Python library** for all
host-side robot interaction (protocol v2).  It provides a layered architecture:

- **`NezhaProtocol`** (`robot/protocol.py`) — serial port owner; all v2 wire I/O
- **`Nezha`** (`robot/nezha.py`) — high-level driver: motion, config, sensors
- **`sensors/`** — OTOS, encoders, camera tracker, color, odometry
- **`nav/`, `path/`, `controllers/`** — navigation, path planning, path controllers
- **`kinematics/`** — differential-drive kinematics

All robot interaction goes through this library.  Tests live in `host/tests/`
(library, 409 tests) and `tests/` (firmware-logic, ~600 tests).

**Run the full suite** (1012 tests, ~1 s):

```sh
uv run --with pytest python -m pytest -q
```

**Smooth-driving quick reference**:
- Discrete moves: `nezha.speed_for_distance(left, right, mm)` — blocking `D`
  command; no watchdog; waits for `EVT done D`.
- Continuous driving: `nezha.stream_drive(speeds, duration_s=N)` — generator;
  sends periodic `S` keepalives; firmware watchdog window is 500 ms (default).

**Calibration tool** (bench, hardware required):

```sh
uv run python tests/calibrate/calibrate_linear.py
```

Run from the repo root; `aprilcam` is provided by the `calibrate` uv dep group
(included in `default-groups` — no extra flags needed).

Full library documentation: **[host/robot_radio/README.md](host/robot_radio/README.md)**

## Raising Issues
Any issues regarding the micro:bit are gathered on the [lancaster-university/codal-microbit-v2](https://github.com/lancaster-university/codal-microbit-v2) repository. Please raise yours there too.

# Installation
You need some open source pre-requisites to build this repo. You can either install these tools yourself, or use the docker image provided below.

- [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
- [Git](https://git-scm.com)
- [CMake](https://cmake.org/download/)
- [Python 3](https://www.python.org/downloads/)

We use Ubuntu Linux for most of our tests. You can also install these tools easily through the package manager:

```
    sudo apt install gcc
    sudo apt install git
    sudo apt install cmake
    sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi
```

## macOS clean setup (recommended)

Use these exact commands to avoid mixed/incomplete ARM toolchains:

```
    brew uninstall arm-none-eabi-gcc arm-none-eabi-binutils
    brew install --cask gcc-arm-embedded
    brew install uv
```

If `arm-none-eabi-gcc` is not found after installation, add links once:

```
    ln -s /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-gcc /opt/homebrew/bin/arm-none-eabi-gcc
    ln -s /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-g++ /opt/homebrew/bin/arm-none-eabi-g++
    ln -s /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-ar /opt/homebrew/bin/arm-none-eabi-ar
    ln -s /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-ranlib /opt/homebrew/bin/arm-none-eabi-ranlib
    ln -s /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-objcopy /opt/homebrew/bin/arm-none-eabi-objcopy
    ln -s /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-size /opt/homebrew/bin/arm-none-eabi-size
```

## Python + UV setup

From the repository root:

```
    uv venv
    uv sync
```

This installs Python modules used by the build and test tooling:

- `pyserial` — reads each device's `DEVICE:` type to avoid flashing the relay

## mbdeploy setup

`mbdeploy` is the standalone deploy package in the `mbdeploy/` directory.
Install it once with pipx so it is available on `$PATH` in its own venv:

```
    pipx install --editable ./mbdeploy
```

Or use the justfile recipe:

```
    just mbd-install
```

### mbdeploy subcommands

```
    mbdeploy build               # compile MICROBIT.hex via build.py
    mbdeploy build --clean       # clean build
    mbdeploy list                # list connected micro:bit devices (UID, port, name)
    mbdeploy probe               # write/update config/devices.json with connected devices
    mbdeploy deploy              # auto-detect the robot and flash MICROBIT.hex
    mbdeploy deploy 1            # flash the device with enum 1 (see 'mbdeploy list')
    mbdeploy deploy --build 1    # build first, then flash device 1
```

### Target selectors

A target can be specified as any of:
- **Enum** — `1`, `2`, etc. (position in `mbdeploy list` output)
- **5-char device name** — e.g. `ZUVUB` (the micro:bit's board name)
- **Serial path** — e.g. `/dev/tty.usbmodem…`
- **Full UID** — the 48-hex-char pyOCD unique ID from `mbdeploy list`

## Just recipes

The repository includes a `justfile` to run common setup/build/deploy workflows.

```
    just --list
    just uv-sync
    just mbd-install        # install/reinstall mbdeploy via pipx
    just build              # compile MICROBIT.hex
    just build-clean        # clean compile
    just list               # list connected micro:bit devices
    just probe              # write/update config/devices.json
    just deploy             # auto-detect the robot and flash MICROBIT.hex
    just build-deploy       # build then deploy
```

## Yotta
For backwards compatibility with [microbit-samples](https://github.com/lancaster-university/microbit-samples) users, we also provide a yotta target for this repository.

## Docker
You can use the [Dockerfile](https://github.com/lancaster-university/microbit-v2-samples/blob/master/Dockerfile) provided to build the samples, or your own project sources, without installing additional dependencies.

Run the following command to build the image locally; the .bin and .hex files from a successful compile will be placed in a new `out/` directory:

```
    docker build -t microbit-tools --output out .
```

To omit the final output stage (for CI, for example) run without the `--output` arguments:

```
    docker build -t microbit-tools .
```

# Building
- Clone this repository
- In the root of this repository type `uv run python3 build.py`
- The hex file will be built `MICROBIT.hex` and placed in the root folder.


# Developing
You will find a simple main.cpp in the `source` folder which you can edit. CODAL will also compile any other C/C++ header files our source files with the extension `.h .c .cpp` it finds in the source folder.

The `samples` folder contains a number of simple sample programs that utilise you may find useful.

## Developer codal.json

There is an example `coda.dev.json` file which enables "developer builds" (clones dependencies from the latest commits, instead of the commits locked in the `codal-microbit-v2` tag), and adds extra CODAL flags that enable debug data to be printed to serial.
To use it, simply copy the additional json entries into your `codal.json` file, or you can replace the file completely (`mv coda.dev.json codal.json`).

# Debugging

See **[docs/debugging.md](docs/debugging.md)** for the full guide — the `just`
recipes (`just debug`, `just gdb`, `just commander`, `just erase`), the pyOCD/SWD
hardware path, agent-oriented (non-interactive) workflows, and recovery from an
APPROTECT-locked chip.

If you are using Visual Studio Code, there is a working debugging environment already set up for you, allowing you to set breakpoints and observe the micro:bit's memory. To get it working, follow these steps:

1. Install either [OpenOCD](http://openocd.org) or [PyOCD](https://github.com/pyocd/pyOCD).
2. Install the [`marus25.cortex-debug` VS Code extension](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug).
3. Build your program.
4. Click the Run and Debug option in the toolbar.
5. Two debugging options are provided: one for OpenOCD, and one for PyOCD. Select the correct one depending on the debugger you installed.

This should launch the debugging environment for you. To set breakpoints, you can click to the left of the line number of where you want to stop.

# Compatibility
This repository is designed to follow the principles and APIs developed for the first version of the micro:bit. We have also included a compatibility layer so that the vast majority of C/C++ programs built using [microbit-dal](https://www.github.com/lancaster-university/microbit-dal) will operate with few changes.

# Documentation
API documentation is embedded in the code using doxygen. We will produce integrated web-based documentation soon.
